//
//      Controls.cpp
//

#include "RadioConfig.h"
#include "ICOM_IC-905_CIV.h"
#include <CIVmaster.h>
#include "Controls.h"

#ifdef USE_RA8875
    extern RA8875 tft;
#else
    extern RA8876_t3 tft;
#endif

extern uint8_t display_state; // something to hold the button state for the display pop-up window later.
extern uint8_t curr_band;     // global tracks our current band setting.
extern uint64_t VFOA;         // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern uint64_t VFOB;
extern struct Modes_List modeList[];
extern struct Band_Memory bandmem[];
extern struct cmdList cmd_List[];
extern struct User_Settings user_settings[];
extern struct Standard_Button std_btn[];
extern struct Label labels[];
extern struct Filter_Settings filter[];
extern struct AGC agc_set[];
extern struct NB nb[];
extern struct Zoom_Lvl zoom[];
extern struct EncoderList encoder_list[];
extern struct TuneSteps tstep[];
extern uint8_t user_Profile;
extern Metro popup_timer; // used to check for popup screen request
extern Metro TX_Timeout;  // RunawayTX timeout
extern uint8_t popup;
extern volatile int64_t Freq_Peak;
extern void set_MF_Service(uint8_t client_name);
extern void unset_MF_Service(uint8_t client_name);
extern uint8_t MF_client;  // Flag for current owner of MF knob services
extern void touchBeep(bool enable);
extern bool MeterInUse; // S-meter flag to block updates while the MF knob has control
extern Metro MF_Timeout;
extern bool MF_default_is_active;
extern uint8_t default_MF_slot;
extern int32_t ModeOffset;
extern uint16_t fft_size;
extern int16_t fft_bins;

#ifdef USE_FREQ_SHIFTER
    extern AudioEffectFreqShiftFD_OA_F32 FFT_SHIFT_I;
    extern AudioEffectFreqShiftFD_OA_F32 FFT_SHIFT_Q;
#endif
#ifdef USE_FFT_LO_MIXER
    extern RadioIQMixer_F32 FFT_LO_Mixer_I;
    extern RadioIQMixer_F32 FFT_LO_Mixer_Q;
#endif
//extern float pan;
float pan;
extern uint64_t xvtr_offset; // LO to apply to actual PLL freq
extern int16_t rit_offset;        // global RIT offset value in Hz.
extern int16_t xit_offset;        // global XIT offset value in Hz.
extern int16_t rit_offset_last;   // global RIT offset value in Hz.
extern int16_t xit_offset_last;   // global RIT offset value in Hz.
extern void update_icon_outline(void);
extern void ringMeter(int val, int minV, int maxV, int16_t x, int16_t y, uint16_t r, const char* units, uint16_t colorScheme, uint16_t backSegColor, int16_t angle, uint8_t inc);
extern uint8_t getRadioMode(void);
extern CIV     civ;
extern CIVresult_t writeMsg (const uint8_t deviceAddr, const uint8_t cmd_body[], const uint8_t cmd_data[],writeMode_t mode);
extern uint8_t check_CIV(uint32_t time_current_baseloop);
extern uint8_t Check_radio(void);
extern uint8_t radio_mode;         // mode from radio messages
extern uint8_t radio_filter;       // filter from radio messages
extern uint8_t radio_data;       // filter from radio messages

const String retValStr[7] = {
    "CIV_OK",
    "CIV_OK_DAV",
    "CIV_NOK",
    "CIV_HW_FAULT",
    "CIV_BUS_BUSY",
    "CIV_BUS_CONFLICT",
    "CIV_NO_MSG"
};

void changeBands(int8_t direction);
void pop_win_up(uint8_t win_num);
void pop_win_down(uint8_t win_num);
void Mute();
void Menu();
void Display();
void Band(uint8_t new_band);
void BandDn();
void BandUp();
void Notch();
void Spot();
void Enet();
void setNR();
void setNB(int8_t toggle);
void Xmit(uint8_t state);
void Ant();
void Fine();
void Rate(int8_t dir);
void setMode(int8_t dir);
void AGC(int8_t dir);
void Filter(int8_t dir);
void ATU();
void Split();
void setXIT(int8_t toggle);
void XIT(int8_t delta);
void setRIT(int8_t toggle);
void RIT(int8_t delta);
void Preamp(int8_t toggle);
void setAttn(int8_t toggle);
void VFO_AB();
void Attn(int8_t delta);
void setAFgain(int8_t toggle);
void AFgain(int8_t delta);
void setRFgain(int8_t toggle);
void RFgain(int8_t delta);
void setRefLevel(int8_t toggle);
void NBLevel(int8_t delta);
void RefLevel(int8_t newval);
void TouchTune(int16_t touch_Freq);
void selectStep(uint8_t fndx);
void selectAgc(uint8_t andx);
void clearMeter(void);
void setMeter(uint8_t id);
void Zoom(int8_t dir);
void setZoom(int8_t toggle);
void PAN(int8_t delta);
void setPAN(int8_t toggle);
void digital_step_attenuator_PE4302(int16_t _attn); // Takes a 0 to 100 input, converts to the appropriate hardware steps such as 0-31dB in 1 dB steps
void setEncoderMode(uint8_t role);

//
//----------------------------------- Skip to Ham Bands only ---------------------------------
//
// Increment band up or down from present.   To be used with touch or physical band UP/DN buttons.
// A alternate method (not in this function) is to use a band button or gesture to do a pop up selection map.
// A rotary encoder can cycle through the choices and push to select or just touch the desired band.
//
// --------------------- Change bands using database -----------------------------------
// Returns 0 if cannot change bands
// Returns 1 if success

// For CIV ops, call CHeck_Radio periodically to prevent message buffer filling up.  Seen TXchk * 5 is a clue this has happened
// In this function many requests are queued up, it will overflow before returning to the main loop to process Check_Radio().
COLD void changeBands(int8_t direction) // neg value is down.  Can jump multiple bandswith value > 1.
{
    int8_t target_band;
    // TODO search bands column for match to account for mapping that does not start with 0 and bands could be in odd order and disabled.

    DPRINTF("\nchangeBands: Incoming Band is "); DPRINTLN(bandmem[curr_band].band_name); DPRINTF("  Current Freq is "); DPRINT(VFOA); DPRINTF("  Current Last_VFOA is "); DPRINTLN(bandmem[curr_band].vfo_A_last);

    Split(0);

    target_band = bandmem[curr_band].band_num + direction;
    // target_band = curr_band + direction;
    
    DPRINTF("changeBands: Proposed Target Band is "); DPRINTLN(bandmem[target_band].band_name);

    uint16_t top_band    = BANDS-1;
    uint16_t bottom_band = 0;

    // See if the proposed new band is enabled in the bandmap or not.  If not skip to the next one and check again until true.
    while (1)  // loop around up or down until we find a valid band to switch to.
    {
        if (target_band > top_band)
            target_band = bottom_band;

        if (target_band < bottom_band) // bottom
            target_band = top_band;

        if (bandmem[target_band].bandmap_en)
        {
            DPRINTF("changeBands: Target band index "); DPRINT(target_band); DPRINTF(" is in the Bandmap. Target band is "); DPRINTLN(bandmem[target_band].band_name);
            break;
        }
        else
        {
            DPRINTF("changeBands: Target band "); DPRINT(bandmem[target_band].band_name); DPRINTLNF(" NOT in Bandmap. Trying next Band.");
            target_band += direction; // maintain change direction up or down.
            if (direction == 0) 
                target_band +=1;  // force a search of current is invalid, possible if VFO set externally to disabled band
        }
    }

    curr_band = target_band; // We have a good band so can new band
    DPRINTF("changeBands: curr_band is "); DPRINTLN(bandmem[curr_band].band_name); 

    if (direction != 0)
    {
        VFOA = bandmem[curr_band].vfo_A_last; // last used frequencies
        DPRINTF("changeBands: Direction not 0, Last used VFOA is "); DPRINTLN(VFOA);
    }
    else
    {
        if (!find_new_band(VFOA, curr_band))  // returns 0 when out of band
        {
            VFOA = bandmem[curr_band].vfo_A_last;   // keep last valid frequency
            DPRINTF("changeBands: after find new band, Last used VFOA is "); DPRINTLN(VFOA);
        }
    }
    DPRINTF("changeBands: New Band is "); DPRINT(bandmem[curr_band].band_name); DPRINTF("  New VFOA is "); DPRINTLN(VFOA);
    
    // Calculate frequency difference between the designated xvtr IF band's lower edge and the current VFO band's lower edge (the LO frequency).
    if (bandmem[curr_band].xvtr_IF)
        xvtr_offset = bandmem[curr_band].edge_lower - bandmem[bandmem[curr_band].xvtr_IF].edge_lower; // if band is 144 then PLL will be set to VFOA-xvtr_offset
    else
        xvtr_offset = 0;
    DPRINTF("changeBands: xvtr_offset is "); DPRINTLN(xvtr_offset);

    DPRINTLNF("changeBands: Set RIT if On");

/*  ToDo - Fix up RIT and XIT later
    if (bandmem[curr_band].RIT_en)
        setRIT(1); // turn on if it was on before.   Also calls selectFrequency(0);
    else
        setRIT(0); // turn off if it was off before on this new band
*/
    selectFrequency(0);
    
    // converts the current band number to a pattern which is then applied to a group of GPIO pins.
    // You can edit the patern for each band in RadioConfig.h
    Band_Decode_Output(curr_band);

    // Example how and when to output new Band Decoder pattern on output IO pins. Pins assignments TBD.
    // Set the new band decoder output pattern for this band change

    DPRINTLNF("changeBands: Set other related band settings");
    // Split(0);
    
    //selectBandwidth(bandmem[curr_band].filter);
    delay(200);
    Check_radio();  //Radio will send back a freq if initialed by the remote side so collect it here
    delay(20);
    get_Preamp_from_Radio(); // sync up with radio
    //Preamp(-1); // -1 sets to database state. 2 is toggle state. 0 and 1 are Off and On.  Operate relays if any.
    //Check_radio();
    get_Attn_from_Radio();  //sync up with radio
    //Check_radio();
    //setAttn(-1); // -1 sets to database state. 2 is toggle state. 0 and 1 are Off and On.  Operate relays if any.
    get_AGC_from_Radio();
    //Check_radio();
    //selectAgc(bandmem[curr_band].agc_mode);
    get_Mode_from_Radio();
    //Check_radio();
    //send_Mode_to_Radio(bandmem[curr_band].mode_A); // set to last known mode on this band

    RefLevel(0); // 0 just updates things to be current value
    RFgain(0);
    AFgain(0);
    NBLevel(0); // 0 just updates things to be current value
    ATU(-1);    // -1 sets to database state. 2 is toggle state. 0 and 1 are Off and On.
    
    //delay(10);
    // Rate(0); Not needed
    // Ant() when there is hardware to setup in the future
    // ATU() when there is hardware to setup in the future
    //
    //    insert any future features, software or hardware, that need to be altered
    //
    user_settings[user_Profile].last_band = curr_band;
    user_settings[user_Profile].sub_VFO = VFOB;
    
    //write_db_tables();  // Save to SD card
    displayRefresh();

    DPRINTLNF("changeBands: Complete\n");
}

//
//  -----------------------   Button Functions --------------------------------------------
//   Called by Touch, Encoder, or Switch events
//

// ---------------------------setMode() ---------------------------
//   selects old or new value and updates buttons, labels to match
//
//   Input: 0 = step to next based on last direction (starts upwards).  Ramps up then down then up.
//          1 = step up 1 
//         -1 = step down 1 
//          2 = use last in dB
//          3 = set from dB but do not send to radio (read radio and set dB to match)
//
COLD void setMode(int8_t dir)
{
    static int8_t direction = 1;
    int8_t _mndx;

    _mndx = (int8_t)bandmem[curr_band].mode_A; // get current mode

    if (dir != 2)
    {
        // 1. Limit to allowed step range
        // 2. Cycle up and at top, cycle back down, do nto roll over.
        if (_mndx <= 0)
        {
            _mndx     = 0; // mode = 0 then change to -1 will add direction == +1 to get 0 later.
            direction = 1; // cycle upwards
        }

        if (_mndx >= MODES_NUM - 1)
        {
            _mndx     = MODES_NUM - 1;
            direction = -1; // go downward
        }

        if (dir == 0)
            _mndx += direction; // Index our step up or down, if dir == 0 then no change to current value
        else
            _mndx += dir; // forces a step higher or lower then current

        if (_mndx > MODES_NUM - 1) // limit in case of encoder countssetMode
            _mndx = MODES_NUM - 1;

        if (_mndx < 0) // limit in case of encoder counts
            _mndx = 0;    

        bandmem[curr_band].mode_A = (uint8_t)_mndx; // store it
    }

    // digital modes only allow AGC Fast
    switch (modeList[bandmem[curr_band].mode_A].mode_num) // loop though the list look for these modes numbers
    {
        case 0x22:  bandmem[curr_band].filter_A = FILT1; // in addition to mode restrictions, if mode is DD then only FIL1 allowed
                    //Filter(2);
        case 0x17: // these modes only allow FAST so do not change radio, just update local buttons, labels  They do allow filter changes
        case 0x23:
        case 0x05:  bandmem[curr_band].agc_mode = AGC_FAST;  // force state to FAST, radio does not auto-report AGC changes
    }

    if (dir < 3)  // Only update radio for local button pushes
        send_Mode_to_Radio((uint8_t) _mndx); // Select the mode for the Active VFO

    displayMode();
    //selectFrequency(0); // Call in case a mode change requires a frequency offset
    get_AGC_from_Radio();  // let the radio update to the per mode AGC setting it has
    get_Attn_from_Radio(); // radio wont tell us it changed so have to ask
    get_Preamp_from_Radio(); // radio wont tell us it changed so have to ask
}

// ---------------------------Filter() ---------------------------
//   selects old or new value and updates buttons, labels to match
//
//  Input: 0 = step to next based on last direction (starts upwards).  Ramps up then down then up.
//          1 = step up 1 filter (wider)
//         -1 = step down 1 filter (narrower)
//          2 = use last filter width used in this mode (from modeList table)
//          3 = set from dB but do not send to radio (read radio and set dB to match)
//
//  This is mode-aware. In non-CW modes we will only cycle through SSB width filters as set in the filter tables
//
// FILTER button
COLD void Filter(int8_t dir)
{
    static int8_t direction = 1;
    int8_t _bw             = bandmem[curr_band].filter_A; // Change Bandwidth  - cycle down then back to the top

    if (dir != 2)
    {
        // 1. Limit to allowed step range
        // 2. Cycle up and at top, cycle back down, do not roll over.
        if (_bw <= FILT1)
        {
            _bw       = FILT1; // mode = 0 then change to -1 will add direction == +1 to get 0 later.
            direction = 1; // cycle upwards
        }

        if (_bw >= FILT3)
        {
            _bw       = FILT3;
            direction = -1; // go downward
        }

        if (dir == 0)
            _bw += direction; // Index our step up or down, if dir == 0 then no change to current value
        else
            _bw += dir; // forces a step higher or lower then current

        if (_bw > FILT3) // limit in case of encoder counts setMode
            _bw = FILT3;

        if (_bw < FILT1) // limit in case of encoder counts
            _bw = FILT1;

    }
    
    // DD mode only allow AGC Fast
    if (modeList[bandmem[curr_band].mode_A].mode_num == 0x22)
    {
        _bw = FILT1;
        dir =3;  // do nto send to radio, it is already there in DD mode
    }
    modeList[bandmem[curr_band].mode_A].Width = bandmem[curr_band].filter_A = (uint8_t) _bw;

    DPRINTF("Set Filter to "); DPRINTLN(filter[bandmem[curr_band].filter_A].Filter_name);
    if (dir < 3)
        send_Mode_to_Radio(bandmem[curr_band].mode_A);
    
    displayFilter();
}

// ---------------------------Rate() ---------------------------
//   Input: 0 = step to next based on last direction (starts upwards).  Ramps up then down then up.
//          1 = step up 1 tune rate
//         -1 = step down 1 tune step rate
//      If FINE is OFF, we will not use 1Hz. If FINE = ON we only use 1 and 10Hz steps.

// RATE button
COLD void Rate(int8_t dir)
{
    static int direction = 1;
    int _tstep           = bandmem[curr_band].tune_step;

    if (user_settings[user_Profile].fine == 0)
    {
        // 1. Limit to allowed step range
        // 2. Cycle up and at top, cycle back down, do nto roll over.
        if (_tstep <= 1)
        {
            _tstep    = 1;
            direction = 1; // cycle upwards
        }

        if (_tstep >= 3) // TS_STEPS-1)
        {
            _tstep    = 3; // TS_STEPS-1;
            direction = -1;
        }

        if (dir == 0)
            _tstep += direction; // Index our step up or down
        else
            _tstep += dir; // forces a step higher or lower then current

        if (_tstep >= 3) // TS_STEPS-1)   // ensure we are still in range
            _tstep = 3;  // TS_STEPS - 1;  // just in case it over ranges, bad stuff happens when it does
        if (_tstep < 1)
            _tstep = 1; // just in case it over ranges, bad stuff happens when it does
    }

    if (user_settings[user_Profile].fine && dir == -1) // 1 Hz steps
        bandmem[curr_band].tune_step = 0;              // set to 1 hz steps
    else if (user_settings[user_Profile].fine && dir == 1)
        bandmem[curr_band].tune_step = 1; // normally swiped is +1 or -1
    else if (user_settings[user_Profile].fine && dir == 0)
    {
        if (_tstep > 0)
            _tstep = 0;
        else
            _tstep = 1;
        bandmem[curr_band].tune_step = _tstep;
    }
    else
        bandmem[curr_band].tune_step = _tstep; // Fine tunig mode is off, allow all steps 10hz and higher

    // DPRINT("Set Rate to ");
    // DPRINTLN(tstep[bandmem[curr_band].tune_step].ts_name);
    displayRate();
}

// AGC button
// ---------------------------AGC() ---------------------------
//   selects old or new value and updates buttons, labels to match
//.
//   Input: 0 = step to next based on last direction (starts upwards).  Ramps up then down then up.
//          1 = step up 1 
//         -1 = step down 1 
//          2 = use last in dB
//          3 = set from dB but do not send to radio (read radio and set dB to match)
//
COLD void AGC(int8_t dir)
{
    static int8_t direction = 1;
    int8_t _agc = bandmem[curr_band].agc_mode;
    
    if (dir < 2)
    {
        // 1. Limit to allowed step range
        // 2. Cycle up and at top, cycle back down, do not roll over.
        if (_agc <= AGC_FAST)
        {
            _agc       = AGC_FAST; // mode = 0 then change to -1 will add direction == +1 to get 0 later.
            direction = 1; // cycle upwards
        }

        if (_agc >= AGC_SLOW)
        {
            _agc       = AGC_SLOW;
            direction = -1; // go downward
        }

        if (dir == 0)
            _agc += direction; // Index our step up or down, if dir == 0 then no change to current value
        else
            _agc += dir; // forces a step higher or lower then current

        if (_agc > AGC_SLOW) // limit in case of encoder counts setMode
            _agc = AGC_SLOW;

        if (_agc < AGC_FAST) // limit in case of encoder counts
            _agc = AGC_FAST;
            
        bandmem[curr_band].agc_mode = (uint8_t) _agc;
    }
    
    // digital modes only allow AGC Fast
    switch (modeList[bandmem[curr_band].mode_A].mode_num) // loop though the list look for these modes numbers
    {
        case 0x17: // these modes only allow FAST so do nto change radio, just update local buttons, labels
        case 0x22:
        case 0x23:
        case 0x05:  bandmem[curr_band].agc_mode = AGC_FAST;  // force state to FAST, radio does not auto-report AGC changes
                    dir = 3;  // send only only if not a digital mode - radio is already in FAST in digital modes
                    DPRINTLNF("AGC(): Changed AGC to FAST for Digital Modes);");
                    break;
    }
    
    // dir  > 1 just use the unchanged dB value.  
    // dir 3 skip sending to radio, just syncing our state to the radio's state
    if (dir < 3)
        send_AGC_to_Radio();
    
    DPRINTF("Set AGC to "); DPRINTLN(agc_set[bandmem[curr_band].agc_mode].agc_name);
    sprintf(std_btn[AGC_BTN].label, "%s", agc_set[bandmem[curr_band].agc_mode].agc_name);
    sprintf(labels[AGC_LBL].label, "%s", agc_set[bandmem[curr_band].agc_mode].agc_name);
    displayAgc();
}

// MUTE button
COLD void Mute()
{
    // float _afLevel = (float) user_settings[user_Profile].afGain/100;

    if (user_settings[user_Profile].spkr_en)
    {
        if (!user_settings[user_Profile].mute)
        {
            //RampVolume(0.0f, 1); //     0 ="No Ramp (instant)"  // loud pop due to instant change || 1="Normal Ramp" // graceful transition between volume levels || 2= "Linear Ramp"
            user_settings[user_Profile].mute = ON;
        }
        else
        { // codec1.muteHeadphone();
            // RampVolume(_afLevel, 1);  //     0 ="No Ramp (instant)"  // loud pop due to instant change || 1="Normal Ramp" // graceful transition between volume levels || 2= "Linear Ramp"
            user_settings[user_Profile].mute = OFF;
            AFgain(0);
        }
        displayMute();
    }
}

// MENU
COLD void Menu()
{
    pop_win_up(SPECTUNE_BTN);

    // tft.fillRect(t_ptr->bx, t_ptr->by, t_ptr->bw, t_ptr->bh, RA8875_BLACK);
    tft.setFont(Arial_24);
    tft.setTextColor(BLUE);
    tft.setCursor(CENTER, CENTER, true);
    tft.print(F("this is a future keyboard"));
    delay(1000);

    pop_win_down(SPECTUNE_BTN); // remove window, restore old screen info, clear popup flag and timer
    displayMenu();
    DPRINTLN("Menu Pressed");
}

// VFO A/B - swap VFO A and B values.
COLD void VFO_AB(void)
{
    // feedback beep
    touchBeep(true); // a timer will shut it off.

    // collect some settings in prep for swapping
    uint64_t old_VFOA  = VFOA;
    uint8_t old_A_mode = bandmem[curr_band].mode_A;
    // uint8_t  old_A_band = curr_band;
    uint64_t old_VFOB  = user_settings[user_Profile].sub_VFO;
    uint8_t old_B_mode = user_settings[user_Profile].sub_VFO_mode;

#ifdef USE_RS_HFIQXXX
    // Compute the band index for the new target band an ensure it is in limits
    // if (RS_HFIQ.find_new_band(old_VFOB, &curr_band))  // return the updated band index for the new freq
#else
    if (find_new_band(old_VFOB, curr_band))
#endif
    {
        // all good, now start swapping
        // DPRINT("\nStart Swapping -  VFO A: ");
        // DPRINT(old_VFOB);
        // DPRINT("  VFO A Mode: ");
        // DPRINT(old_B_mode);
        // DPRINT(" VFO A band: ");
        // DPRINTLN(curr_band);
        VFOA = bandmem[curr_band].vfo_A_last = old_VFOB; // Update VFOA to new freq, then update the band index to match
        bandmem[curr_band].mode_A            = old_B_mode;
        send_Mode_to_Radio(old_B_mode); // copy to VFOA mode and apply

        // DPRINT("Stash sub_VFO values - VFO B: ");
        // DPRINT(old_VFOA);
        // DPRINT("  VFO B Mode: ");
        // DPRINT(old_A_mode);
        // DPRINT(" VFO B band: ");
        // DPRINTLN(old_A_band);
        VFOB = user_settings[user_Profile].sub_VFO = old_VFOA;   // save A into the database
        user_settings[user_Profile].sub_VFO_mode   = old_A_mode; // Udpate VFOB
    }
    selectFrequency(0);
    changeBands(0);
    displayVFO_AB();
    displayMode();
    DPRINT("Set VFO_A to ");
    DPRINTLN(VFOA);
    DPRINT("Set VFO_B to ");
    DPRINTLN(VFOB);
}

// ----------------------------------- setAttn ---------------------------------------------------------
//  setAttn  - valid on bands < 2400 for IC-905
//   -1 sets attenuator state to current database value. Used for startup or changing bands.
//    0 sets attenuator state off
//    1 sets attenuator state on
//    2 toggles attenuator state
//    3 set attenuator state to current database value but does nto send to radio (likely came from radio)
//
COLD void setAttn(int8_t toggle)
{
    DPRINTF("setAttn: toggle = "); DPRINTLN(toggle);
    DPRINTF("setAttn: Attn start state = "); DPRINTLN(bandmem[curr_band].attenuator);

    CIVresult_t CIVresultL;

    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (bandmem[curr_band].attenuator == ATTN_ON) // toggle the attenuator tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // toggle is 1, turn on Attn
    {
        bandmem[curr_band].attenuator     = ATTN_ON; // set the attenuator tracking state to ON
    }

    if (toggle == 0 || curr_band > BAND1296)  // preamp and atten not available on IC905 on bands above 1296
    {
        bandmem[curr_band].attenuator     = ATTN_OFF; // set attenuator tracking state to OFF
    }

    MeterInUse = false;

    if (bandmem[curr_band].preamp == PREAMP_ON && bandmem[curr_band].attenuator == ATTN_ON)   // turn off if the preamp is on
        bandmem[curr_band].preamp = PREAMP_OFF;


    DPRINTF("setAttn 2: toggle = "); DPRINTLN(toggle);
    DPRINTF("setAttn 2: ATTN start state = "); DPRINTLN(bandmem[curr_band].attenuator);
    DPRINTF("setAttn:2: Preamp state = "); DPRINTLN(bandmem[curr_band].preamp);


    // Reading from the radio we just want to update database and screen and not repeat back to radio.
    // 0 = no change to set attenuator level to value in database for this band
    
    if (curr_band < BAND2400)
    {
        if (toggle < 3)
        {
            if (bandmem[curr_band].attenuator) 
            {
                delay(20);
                CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_ATTN_ON].cmdData), CIV_D_NIX, CIV_wChk);
                DPRINTF("setAttn: Send to Radio ON: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
            }
            else
            {
                CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_ATTN_OFF].cmdData), CIV_D_NIX, CIV_wChk);
                DPRINTF("setAttn: Send to Radio OFF: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
            }
        }
    }
    else  // the band is > 1296
    {
        DPRINTF("setAttn: Skipping for bands > 1296 - curr band is "); DPRINTLN(bandmem[curr_band].band_name);
    }

    displayAttn();
    displayPreamp();

    DPRINTF("setAttn: Set Attenuator to "); DPRINTLN(bandmem[curr_band].attenuator);
    // DPRINTF("Set Attenuator Relay to "); DPRINT(bandmem[curr_band].attenuator_byp); DPRINTF(" Attn_dB is "); DPRINTLN(bandmem[curr_band].attenuator_dB);
    // DPRINTF(" and Ref Level is "); DPRINTLN(Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_floor);
}

/*******************************************************************************
 * Function Name: Attn()
 ********************************************************************************
 *
 * Summary:
 * Main function performs following functions:
 * 1: Configures the solid state attenuator by shifting 16 bits of address and
 *    attn level in LSB first.
 *
 * Parameters:
 *  attn = attenuation level to set in range of 0 to 100% (0 to 31 (in dB))
 *
 * Return:
 *  None.
 *
 *******************************************************************************/
COLD void Attn(int8_t delta)
{
    int8_t _attn = bandmem[curr_band].attenuator_dB;

    _attn += delta * 10;

    if (_attn > 10) // Keep in 0-100 range 10 for IC-905 on bands < 2400
    {
        bandmem[curr_band].attenuator_dB = _attn = 10;
        setAttn(1);
    }
    if (_attn <= 0 || curr_band > BAND1296)  // skip for high bands and force off
    {
        bandmem[curr_band].attenuator_dB = _attn = 0;    
        setAttn(0);
    }
    
    DPRINT("Setting attenuator value to "); DPRINTLN(bandmem[curr_band].attenuator_dB);
   
    //displayAttn(); // update the button value
}

// ------------------------PREAMP button  - valid on bands < 2400 for IC-905 -----------------------
//   0 sets Preamp state off
//   1 sets Preamp state on
//   2 toggles Preamp state
//   -1 or any value other than 0-2 sets Preamp state to current database value. Used for startup or changing bands.
//   3 use value from dB but do not send to radio (was likely read from radio) 
//
COLD void Preamp(int8_t toggle)
{
    CIVresult_t CIVresultL;
    
    DPRINTF("Preamp: toggle = "); DPRINTLN(toggle);
    DPRINTF("Preamp: Preamp start state = "); DPRINTLN(bandmem[curr_band].preamp);

    if (toggle == 2) // toggle state
    {
        if (bandmem[curr_band].preamp == PREAMP_ON)
           toggle = 0;
        else
            toggle = 1;
    }
    
    if (toggle == 1) // set to ON
    {
        bandmem[curr_band].preamp = PREAMP_ON;
    }

    if (toggle == 0 || curr_band > BAND1296) // set to OFF
    {
        bandmem[curr_band].preamp = PREAMP_OFF;
    }   
        // any other value of toggle pass through with unchanged state, jsut set the relays to current state
    
    if (bandmem[curr_band].attenuator == ATTN_ON && bandmem[curr_band].preamp ==PREAMP_ON)
        bandmem[curr_band].attenuator = ATTN_OFF;   // turn off if attn is on

    DPRINTF("Preamp 2: toggle = "); DPRINTLN(toggle);
    DPRINTF("Preamp 2: Preamp start state = "); DPRINTLN(bandmem[curr_band].preamp);
    DPRINTF("Preamp:2: ATTN state = "); DPRINTLN(bandmem[curr_band].attenuator);

    // Reading from the radio we just want to update database and screen and not repeat back to radio.
    if (curr_band < BAND2400)
    {
        if (toggle < 3 )
        {
            if (bandmem[curr_band].preamp) 
            {
                CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_PREAMP_ON].cmdData), CIV_D_NIX, CIV_wChk);
                DPRINTF("Preamp: Send to Radio ON: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
            }
            else
            {
                CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_PREAMP_OFF].cmdData), CIV_D_NIX, CIV_wChk);
                DPRINTF("Preamp: Send to Radio OFF: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
            }
        }
    }
    else  // preamp and atten not available on IC905 on bands above 1296
    {
        DPRINTF("Preamp: Skipping for bands > 1296 - curr band is "); DPRINTLN(bandmem[curr_band].band_name);
    }

    displayAttn();
    displayPreamp();
    
    DPRINTF("Preamp: Set Preamp to "); DPRINTLN(bandmem[curr_band].preamp);
}

// RIT button
// Input:
// -1 = Clear MF knob and meter, use current value
//  0 = OFF - turn off button highlight color
//  1 = ON  - turn off button highlight and center pan window
//  2 = Toggle RIT ON/OFF state.
//  3 = Zero RIT offset value
COLD void setRIT(int8_t toggle)
{
    // global rit_offset is used add to VFOA when displaying or reporting or setting frequency.
    // It never changes VFOA value to keep memory and band change complications.
    // Will be toggled between 0 and the offset value when active.
    // This eliminates the need to test for RIT at every VFOA handling point
    if (toggle == 3 && bandmem[curr_band].RIT_en == ON) // Adjust the RIT value if active
    {
        toggle = 1;
    }
    else if (toggle == 3 && bandmem[curr_band].RIT_en == OFF) // Zero the RIT value
    {
        rit_offset_last = rit_offset = 0;
        RIT(0); // update frequency offset
    }

    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (bandmem[curr_band].RIT_en == ON) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Turn on RIT and set to last stored offset
    {
        bandmem[curr_band].RIT_en = ON;
        rit_offset                = rit_offset_last; // restore last used value
        MeterInUse                = true;
        setMeter(RIT_BTN);
        RIT(0);
    }

    if (toggle == 0) // prevent acting on multiple calls
    {
        bandmem[curr_band].RIT_en = OFF;
        rit_offset                = 0; // now clear it
        RIT(0);
    }

    if (toggle == 0 || toggle == -1)
    {
        MeterInUse = false;
        if (toggle != -1) clearMeter();
    }

    // DPRINTF("setRIT: Set RIT ON/OFF to "); DPRINTLN(bandmem[curr_band].RIT_en);
    // DPRINTF("setRIT: Set RIT OFFSET to "); DPRINT(rit_offset); DPRINTF("  rit_offset_last = "); DPRINTLN(rit_offset_last);
}

// RIT offset control
COLD void RIT(int8_t delta)
{

    int16_t _offset;

    if (bandmem[curr_band].RIT_en == ON)
    {
        _offset = rit_offset; // Get cuurent value
        _offset += delta * tstep[user_settings[user_Profile].RIT_ts].step;

        if (_offset > 9999) // Limit the value between -10K and +10K
            _offset = 9999;
        if (_offset < -9999)
            _offset = -9999;

        rit_offset = _offset; // store in the global RIT offset value
        if (bandmem[curr_band].RIT_en == ON)
            rit_offset_last = rit_offset; // only store when RIT is active, prevent false zero for things like delayed calls from S-meter box updates

        // DPRINTF(RIT: Set RIT OFFSET to "); DPRINT(rit_offset); DPRINTF("  rit_offset_last = "); DPRINTLN(rit_offset_last);
    }
    selectFrequency(0); // no base freq change, just correct for RIT offset
    displayFreq();
    displayRIT();
}

// XIT button
// Input:
// -1 = Clear MF knob and meter, use current value
//  0 = OFF - turn off button highlight color
//  1 = ON  - turn off button highlight and center pan window
//  2 = Toggle XIT ON/OFF state.
//  3 = Zero XIT offset value
COLD void setXIT(int8_t toggle)
{
    // global xit_offset is used add to VFOA when displaying or reporting or setting frequency.
    // It never changes VFOA value to keep memory and band change complications.
    // Will be toggled between 0 and the offset value when active.
    // This eliminates the need to test for XIT at every VFO handling point
    if (toggle == 3 && bandmem[curr_band].XIT_en == ON) // Adjust the RIT value if active
    {
        toggle = 1;
    }
    else if (toggle == 3 && bandmem[curr_band].XIT_en == OFF) // Zero the RIT value
    {
        xit_offset_last = xit_offset = 0;
        XIT(0); // update frequency offset
    }

    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (bandmem[curr_band].XIT_en == ON) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Turn on XIT and set to last stored offset
    {
        bandmem[curr_band].XIT_en = ON;
        xit_offset                = xit_offset_last; // restore last used value
        MeterInUse                = true;
        setMeter(XIT_BTN);
        XIT(0);
    }

    if (toggle == 0) // prevent acting on multiple calls
    {
        bandmem[curr_band].XIT_en = OFF;
        xit_offset                = 0; // now clear it
        XIT(0);
    }

    if (toggle == 0 || toggle == -1)
    {
        MeterInUse = false;
        if (toggle != -1) clearMeter();
    }

    if (toggle == 3) // Zero the RIT value
    {
        xit_offset_last = xit_offset = 0;
        XIT(0); // update frequency
    }

    // DPRINTF("setXIT: Set XIT ON/OFF to "); DPRINTLN(bandmem[curr_band].XIT_en);
    // DPRINTF("setXIT: Set XIT OFFSET to "); DPRINT(xit_offset); DPRINTF("  xit_offset_last = "); DPRINTLN(xit_offset_last);
}

// XIT offset control
COLD void XIT(int8_t delta)
{

    int16_t _offset;

    if (bandmem[curr_band].XIT_en == ON)
    {
        _offset = xit_offset; // Get cuurent value
        _offset += delta * tstep[user_settings[user_Profile].XIT_ts].step;                               

        if (_offset > 9999) // Limit the value between +/- 10KHz
            _offset = 9999;
        if (_offset < -9999)
            _offset = -9999;

        xit_offset = _offset; // store in the global XIT offset value
        if (bandmem[curr_band].XIT_en == ON)
            xit_offset_last = xit_offset; // only store when XIT is active, prevent false zero for things like delayed calls from S-meter box updates

        //DPRINTF("XIT: Set XIT OFFSET to "); DPRINT(xit_offset); DPRINTF("  xit_offset_last = "); DPRINTLN(xit_offset_last);
    }
    selectFrequency(0); // no base freq change, just correct for RIT offset
    displayFreq();
    displayXIT();
}

// SPLIT button
//   state = 0 sets Split state off
//   state = 1 sets Split state on
//   state = 2 toggles Split state
COLD void Split(uint8_t state)
{
    if (state == 0)
        bandmem[curr_band].split = OFF;
    if (state == 1)
        bandmem[curr_band].split = ON;
    if (state == 2)
    {
        if (bandmem[curr_band].split == ON)
            bandmem[curr_band].split = OFF;
        else if (bandmem[curr_band].split == OFF)
            bandmem[curr_band].split = ON;
    }
    displaySplit();
    displayFreq();
    // DPRINTF("Set Split to "); DPRINTLN(bandmem[curr_band].split);
}

// DATA ON/Off button
//   state = 0 sets DATA state off
//   state = 1 sets DATA state on
//   state = 2 toggles DATA state
COLD void DATA(uint8_t state)
{
    if (state == 0)
        bandmem[curr_band].data_A = OFF;
    if (state == 1)
        bandmem[curr_band].data_A = ON;
    if (state == 2)
    {
        if (bandmem[curr_band].data_A == ON)
            bandmem[curr_band].data_A = OFF;
        else if (bandmem[curr_band].data_A == OFF)
            bandmem[curr_band].data_A = ON;
    }
    displayDATA();
    displayFreq();
    // DPRINTF("Set Split to "); DPRINTLN(bandmem[curr_band].split);
}

// ATU button
// 0 = OFF  1 = ON   2 = Toggle  -1 = update to database state
COLD void ATU(uint8_t state)
{
    if ((bandmem[curr_band].ATU == ON && state == 2) || state == 0)
        bandmem[curr_band].ATU = OFF;
    else if ((bandmem[curr_band].ATU == OFF && state == 2) || state == 1)
        bandmem[curr_band].ATU = ON;

    //if (bandmem[curr_band].ATU == ON)
        //TwoToneTest = ON; // For test turn Two Tone test on and off.  When off the Mic is enabled.
    //else
        //TwoToneTest = OFF; // For test turn Two Tone test on and off.  When off the Mic is enabled.
    //
    //   Insert any future ATU hardware setup calls
    //

    //displayATU();
    // DPRINTF("Set ATU to "); DPRINTLN(bandmem[curr_band].ATU);
}

// setZoom()
//   toogle = 0 sets Zoom state to current database value. Used for startup or changing bands.
//   toogle = -1 deregisters Zoom MF control
//   toogle = 1 sets Zoom state on
//   toogle = 2 toggles Zoom state
COLD void setZoom(int8_t toggle)
{
    // DPRINT("toggle = "); DPRINTLN(toggle);

    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (user_settings[user_Profile].zoom_level >= 2) // toggle the tracking state
            user_settings[user_Profile].zoom_level = 0;
        else
            user_settings[user_Profile].zoom_level += 1;
        toggle = 1;
    }

    if (toggle == 1) // Set button to on to track as active
    {
        MeterInUse = true;
        setMeter(ZOOM_BTN);
        Zoom(0);
    }

    if (toggle == 0 || toggle == -1)
    {
        if (toggle == 0) Zoom(0);
        MeterInUse = false;
        if (toggle != -1) clearMeter();
    }

    // displayZoom();
    // DPRINTF("Set Zoom to "); DPRINTLN(user_settings[user_Profile].zoom_level);
}

// ---------------------------Zoom() ---------------------------
//   Input: 2 = step to next based on last direction (starts upwards).  Ramps up then down then up.
//          1 = step up 1 (zoomed in more)
//         -1 = step down 1 (zoom out more )
//          0 = use last zoom level used from user profile
//   Zoom levels are x1, x2 and x4  Off is same as x1.
//
COLD void Zoom(int8_t dir)
{
    static int8_t direction = -1;                                     // remember last direction
    int8_t _zoom_Level      = user_settings[user_Profile].zoom_level; // Get last known value from user profile

    if (dir != 0)
    {
        if (dir >= 2)
            dir = 1;

        // 1. Limit to allowed step range
        // 2. Cycle up and at top, cycle back down, do not roll over.
        if (_zoom_Level <= 0)
        {
            _zoom_Level = 0;
            direction   = 1; // cycle upwards
        }
        if (_zoom_Level >= ZOOM_NUM - 1)
        {
            _zoom_Level = ZOOM_NUM - 1;
            direction   = -1;
        }
        if (dir == 0)
            _zoom_Level += direction; // Index our step up or down
        else
            _zoom_Level += dir; // forces a step higher or lower then current
        // Ensure we have legal values
        if (_zoom_Level <= 0)
            _zoom_Level = 0;
        if (_zoom_Level >= ZOOM_NUM - 1)
            _zoom_Level = ZOOM_NUM - 1;
        user_settings[user_Profile].zoom_level = (uint8_t)_zoom_Level; // We have our new table index value
    }

    switch (_zoom_Level)
    {
        #ifdef FFT_1024
                case ZOOMx1:
                    //Change_FFT_Size(1024, sample_rate_Hz);
                    break; // Zoom farthest in
        #endif
        #ifdef FFT_2048
                case ZOOMx2:
                    //Change_FFT_Size(2048, sample_rate_Hz);
                    break; // Zoom farthest in
        #endif
        #ifdef FFT_4096
                case ZOOMx4:
                    //Change_FFT_Size(4096, sample_rate_Hz);
                    break; // Zoom farthest in
        #endif
        default:
            //Change_FFT_Size(FFT_SIZE, sample_rate_Hz);
            break; // Zoom farthest in
    }

    // DPRINTF("Zoom level set to  "); DPRINTLN(zoom[_zoom_Level].zoom_name);
    //displayZoom();
}

// Fine button
COLD void Fine()
{
    extern uint8_t enc_ppr_response;

    if (user_settings[user_Profile].fine == ON)
    {
        user_settings[user_Profile].fine = OFF;
        enc_ppr_response /= 1.4;
    }
    else if (user_settings[user_Profile].fine == OFF)
    {
        user_settings[user_Profile].fine = ON;
        enc_ppr_response *= 1.4;
    }
    Rate(0);
    displayFine();
    displayRate();

    // DPRINTF("Set Fine to "); DPRINTLN(user_settings[user_Profile].fine);
}

// ANT button
// Toggle between antenna 1 and 2.  Turns GPIO_ANT_PIN ON and OFF to control a relay.
COLD void Ant()
{
    if (bandmem[curr_band].ant_sw == 1)
    {
        bandmem[curr_band].ant_sw = 2;
        if (GPIO_ANT_ENABLE)
            digitalWrite(GPIO_ANT_PIN, 1);
    }
    else if (bandmem[curr_band].ant_sw == 2)
    {
        bandmem[curr_band].ant_sw = 1;
        if (GPIO_ANT_ENABLE)
            digitalWrite(GPIO_ANT_PIN, 0);
    }

    displayANT();
    // DPRINTF("Set Ant Sw to "); DPRINTLN(bandmem[curr_band].ant_sw);
}

// AF GAIN button activate control
COLD void setAFgain(int8_t toggle)
{
    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (user_settings[user_Profile].afGain_en) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Set button to on to track as active
    {
        if (user_settings[user_Profile].afGain_en == ON && MeterInUse) // if already on, assume this was called by a long press and we want to turn off the meter and but not the feature.
            clearMeter();
        else
        {
            user_settings[user_Profile].afGain_en = ON; // set the af tracking state to ON
            MeterInUse = true;
            setMeter(AFGAIN_BTN);
        }
    }

    if (toggle == 0 || toggle == -1)
    {
        user_settings[user_Profile].afGain_en = OFF;
        MeterInUse = false;
        if (toggle != -1)
            clearMeter();
    }

    // DPRINT(" AF Gain ON/OFF set to  ");
    // DPRINTLN(user_settings[user_Profile].afGain_en);
    displayAFgain();
}

// AFGain Adjust
//  Input:   0 no change
//          -X Reduce by X%
//          +X Increase by X%
//
//   Request a new volume as a percentage up or down.
//   To jump to Full volume use 100;
//   To jump to Min volume use -100;
//   Normally just ask for +1 or -1
//
COLD void AFgain(int8_t delta)
{
    float _afLevel;

    // LineOutLevel is 0 to 31 with 0-12 clipping.  So 13-31 is usable range.  This scale is inverted.  13 is loudest, 31 lowest output.
    if (user_settings[user_Profile].xmit == OFF)
        _afLevel = user_settings[user_Profile].afGain; // Get last absolute volume setting as a value 0-100
    else
        _afLevel = user_settings[user_Profile].mic_Gain_level; // Get last absolute volume setting as a value 0-100

    // DPRINT(" TEST AF Level requested "); DPRINTLN(_afLevel);

    _afLevel += delta * 2; // convert percentage request to a single digit float

    // DPRINT(" TEST AF Level absolute "); DPRINTLN(_afLevel);

    if (_afLevel > 100) // Limit the value between 0.0 and 1.0 (100%)
        _afLevel = 100;
    if (_afLevel < 1)
        _afLevel = 1; // do not use 0 to prevent divide/0 error

    // LineOutLevel is 0 to 31 with 0-12 clipping.  So 13-31 is usable range.  This scale is inverted.  13 is loudest, 31 lowest output.
    if (user_settings[user_Profile].xmit == OFF)
    {
        user_settings[user_Profile].afGain = _afLevel; // update memory
        // codec1.lineOutLevel(user_settings[user_Profile].lineOut_RX * _afLevel/100); // skip when in TX to act as Mic Level Adjust control
    }
    else // Control Mic Gain and Power out
    {
        //   Temp mic/lineout level control until a real control is created for these
        _afLevel                                   = 100;                  // force to 100%
        user_settings[user_Profile].mic_Gain_level = _afLevel;             // 0 to 100 mic gain range

        DPRINT("Mic Gain (0-63dB)= ");
        DPRINTLN(_afLevel * 0.63);
        DPRINT("Power Out(0-100%)= ");
        DPRINTLN(_afLevel);
        //codec1.muteHeadphone();
    }
    // Convert linear pot to audio taper pot behavior
    // Use new afLevel
   // float val = log10f(_afLevel) / 2.0f;
    // DPRINT(" Volume set to  log = ");
    // DPRINTLN(val);

#ifdef USE_FFT_LO_MIXERxxxx
    AudioNoInterrupts();
    FFT_LO_Mixer_I.frequency((_afLevel)*200.0f);
    FFT_LO_Mixer_Q.frequency((_afLevel)*200.0f);
    AudioInterrupts();
#endif

    // RampVolume handles the scaling. Must set the LineOutLevel to the desired max though.
   //RampVolume((float)val, 2); //     0 ="No Ramp (instant)"  // loud pop due to instant change || 1="Normal Ramp" // graceful transition between volume levels || 2= "Linear Ramp"
    // DPRINT(" Volume set to  ");
    // DPRINTLN(_afLevel);
    displayAFgain();
}

// RF GAIN button activate control
COLD void setRFgain(int8_t toggle)
{
    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (user_settings[user_Profile].rfGain_en) // toggle the attenuator tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Set button to on to track as active
    {
        user_settings[user_Profile].rfGain_en = ON;
        MeterInUse                            = true;
        setMeter(RFGAIN_BTN);
    }

    if (toggle == 0 || toggle == -1)
    {
        user_settings[user_Profile].rfGain_en = OFF;
        MeterInUse                            = false;
        if (toggle != -1)
            clearMeter();
    }

    // DPRINT(" RF Gain ON/OFF set to  ");
    // DPRINTLN(user_settings[user_Profile].rfGain_en);
    displayRFgain();
}

// RF GAIN Adjust
//
//  Input:   0 no change
//          -X Reduce by X%
//          +X Increase by X%
//
COLD void RFgain(int8_t delta)
{
    float _rfLevel;

    _rfLevel = user_settings[user_Profile].rfGain; // Get last absolute setting as a value 0-100

    _rfLevel += delta * 4; // convert percentage request to a single digit float

    if (_rfLevel > 100) // Limit the value between 0.0 and 1.0 (100%)
        _rfLevel = 100;
    if (_rfLevel < 1)
        _rfLevel = 1; // do not use 0 to prevent divide/0 error

    // Store new value as 0 to 100%
    user_settings[user_Profile].rfGain = _rfLevel; // 0 to 100 range, ,linear

    // DPRINT("CodecLine IN level set to ");
    // DPRINTLN(user_settings[user_Profile].lineIn_level * user_settings[user_Profile].rfGain/100);
    // DPRINT("RF Gain level set to  ");
    // DPRINTLN(_rfLevel);
    displayRFgain();
}

// PAN ON/OFF button activate control
// -1 is clear meter- used by MF knob and S-meter box   leave pan window alone.
//  0 is OFF - turn off button highlight and center pan window
//  1 is ON
//  2 is toggle ON/OFF state
//  3 is center the pan window
COLD void setPAN(int8_t toggle)
{
    // DPRINT("PAN toggle = "); DPRINTLN(toggle);

    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (user_settings[user_Profile].pan_state == ON) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Set button to on to track as active
    {
        user_settings[user_Profile].pan_state = ON;
        MeterInUse                            = true;
        setMeter(PAN_BTN);
        PAN(0);
    }

    if (toggle == 0 || toggle == -1 || toggle == 3)
    {
        user_settings[user_Profile].pan_state = OFF;
        MeterInUse                            = false;
        if (toggle != -1) clearMeter();
        if (toggle == 3) // Zero the pan window and turn off on long button press
        {
            pan                                   = 0.0f;
            user_settings[user_Profile].pan_level = 50;
        }
    }
    // DPRINT(" PAN state is  ");
    // DPRINTLN(user_settings[user_Profile].pan_state);
    displayPan();
}

// PAN Adjust
//
//  Input:   0 no change
//          -X Reduce by X%
//          +X Increase by X%
//
COLD void PAN(int8_t delta)
{
    int8_t _panLevel;

    _panLevel = (int8_t)user_settings[user_Profile].pan_level; // Get last absolute setting as a value 0-100
    _panLevel += delta;                                        // convert percentage request to a single digit float

    if (_panLevel >= 100) // Limit the value between 0.0 and 1.0 (100%)
        _panLevel = 100;
    if (_panLevel <= 0)
        _panLevel = 0;

    if (user_settings[user_Profile].pan_state == OFF && delta == 0)
    {
        pan                                   = 0.0f;
        _panLevel                             = 50; // 50 is halfway or 0.0f
        user_settings[user_Profile].pan_level = _panLevel;
    }
    else
    {                                                                           // bins that will not fit on the display can be viewed by shifting our left edge index.  Assuming 1 px per bin
        pan                                   = float(_panLevel - 50) / 100.0f; // 0 is 0.0.  50 is +0.5, -50 is -0.5.
        user_settings[user_Profile].pan_level = _panLevel;
    }

    // DPRINT("Control Change: PAN set to  ");
    // DPRINTLN(user_settings[user_Profile].pan_level-50); // convert to -100 to +100 for UI
    displayPan();
}

// XMIT button
COLD void Xmit(uint8_t state) // state ->  TX=1, RX=0; Toggle =2
{
    //uint8_t mode_idx;

    //mode_idx = bandmem[curr_band].mode_A;

    if ((user_settings[user_Profile].xmit == ON && state == 2) || state == 0) // Transmit OFF
    {
        user_settings[user_Profile].xmit = OFF;
        if (PTT_OUT1 != 255)
            digitalWrite(PTT_OUT1, HIGH);

        // enable line input to pass to headphone jack on audio card, set audio levels
        //TX_RX_Switch(OFF, mode_idx, OFF, OFF, OFF, OFF, 0.5f);
        // int TX,                 // TX == 1, RX == 0
        // uint8_t mode_sel,       // Current VFO mode index
        // float   Mic_On,         // 0.0f(OFF) or 1.0f (ON)
        // float   ToneA,          // 0.0f(OFF) or 1.0f (ON)
        // float   ToneB,          // 0.0f(OFF) or 1.0f (ON)
        // float   TestTone_Vol)   // 0.90 is max, clips if higher. Use 0.45f with 2 tones
        DPRINTLN("XMIT(): TX OFF");
    }
    else if ((user_settings[user_Profile].xmit == OFF && state == 2) || state == 1) // Transmit ON
    {
        user_settings[user_Profile].xmit = ON;
        if (PTT_OUT1 != 255)
            digitalWrite(PTT_OUT1, LOW);

        //TX_Timeout.reset(); // Reset our Runaway TX timer.  Main loop will watch for this to trip calling back here to flip back to RX.

        // enable mic input to pass to line out on audio card, set audio levels
        //if (TwoToneTest)                                         // do test tones
        //    TX_RX_Switch(ON, mode_idx, OFF, OFF, ON, ON, 0.05f); // TestOne_Vol => 0.90 is max, clips if higher. Use 0.45f with 2 tones
        //else if (mode_idx == DATA || mode_idx == DATA_REV)       // Mic on, turn off test tones
        ///   TX_RX_Switch(ON, mode_idx, OFF, ON, OFF, OFF, OFF);  // Turn on USB input, Turn Mic OFF
        //else
        //    TX_RX_Switch(ON, mode_idx, ON, OFF, OFF, OFF, OFF); // Turn Mic input ON, Turn USB IN OFF
        //DPRINTLN("XMIT(): TX ON");
    }
    displayXMIT();
    displayFreq();
    // DPRINT("Set XMIT to "); DPRINTLN(user_settings[user_Profile].xmit);
}

// NB ON/OFF button
// -1 is clear meter- used by MF knob and S-meter box
//  0 is OFF - turn off button highlight and center pan window
//  1 is ON
//  2 is toggle ON/OFF state
COLD void setNB(int8_t toggle)
{
    // char string[80];   // print format stuff
    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (user_settings[user_Profile].nb_en) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }
    if (toggle == 1)
    {
        user_settings[user_Profile].nb_en = ON;
        NBLevel(0);
        MeterInUse = true;
        setMeter(NB_BTN);
    }

    if (toggle == 0)
    {
        user_settings[user_Profile].nb_en = OFF;
        NBLevel(0);
    }

    if (toggle == 0 || toggle == -1)
    {
        MeterInUse = false;
        if (toggle != -1)
            clearMeter();
    }

    // DPRINTF("Set NB to "); DPRINTLN(user_settings[user_Profile].nb_en);
    displayNB();
}

// Adjust the NB level. NB() turn on and off only calling this to initiialize the current level with delta = 0
// NB Level Adjust - range is 1 to 6
//  0 is Use Current value in DB.  If NB_en is OFF, then disable NB.
//  + increments
//  - decrements
COLD void NBLevel(int8_t delta)
{
    int8_t _nbLevel;

    // DPRINT(" TEST NB delta "); DPRINTLN(delta);

    _nbLevel = user_settings[user_Profile].nb_level; // Get last known value

    // DPRINT(" TEST NB Level "); DPRINTLN(_nbLevel);

    _nbLevel += delta; // increment up or down

    /*if (_nbLevel > 100)         // Limit the value
        _nbLevel = 100;
    if (_nbLevel < 0)
        _nbLevel = 0;

    DPRINT(" TEST NB New Level "); DPRINTLN(_nbLevel);
    user_settings[user_Profile].nb_level = _nbLevel;  // We have our new table index value

    // Convert from 0 to 100% to number of valid steps
    _nbLevel *= round(NB_SET_NUM-1 +0.5)/100;
    DPRINT(" TEST NB Converted Level "); DPRINTLN(_nbLevel);
*/
    if (_nbLevel > NB_SET_NUM - 1) // Limit the value
        _nbLevel = NB_SET_NUM - 1;
    if (_nbLevel < 1)
        _nbLevel = 1;

    user_settings[user_Profile].nb_level = _nbLevel; // We have our new table index value

    if (user_settings[user_Profile].nb_en == ON) // Adjust the value if on
    {
    }
    else // NB is disabled so bypass
    {
    }

    // DPRINTF("NB level set to  "); DPRINTLN(_nbLevel);
    displayNB();
}

// NR button
COLD void setNR()
{
    if (user_settings[user_Profile].nr_en > NROFF)
    {
        user_settings[user_Profile].nr_en = NROFF;
    }
    else if (user_settings[user_Profile].nr_en == NROFF)
    {
        user_settings[user_Profile].nr_en = NR1;
    }
    displayNR();
    // DPRINTF("Set NR to "); DPRINTLN(user_settings[user_Profile].nr_en);
}

// Enet button
COLD void Enet()
{
    if (user_settings[user_Profile].enet_output == ON)
        user_settings[user_Profile].enet_output = OFF;
    else if (user_settings[user_Profile].enet_output == OFF)
        user_settings[user_Profile].enet_output = ON;

    displayEnet();
    // DPRINTF("Set Ethernet to "); DPRINTLN(user_settings[user_Profile].enet_output);
}

// Spot button
COLD void Spot()
{
    if (user_settings[user_Profile].spot == OFF)
        user_settings[user_Profile].spot = ON;
    else
        user_settings[user_Profile].spot = OFF;
    // displaySpot();
    // DPRINTF("Set Spot to "); DPRINTLN(user_settings[user_Profile].spot);
}

// REF LEVEL button activate control
COLD void setRefLevel(int8_t toggle)
{
    if (toggle == 2) // toggle if ordered, else just set to current state such as for startup.
    {
        if (std_btn[REFLVL_BTN].enabled) // toggle the tracking state
            toggle = 0;
        else
            toggle = 1;
    }

    if (toggle == 1) // Set button to on to track as active
    {
        std_btn[REFLVL_BTN].enabled = ON;
        if (MF_client != REFLVL_BTN)
        {
            MeterInUse = true;
            setMeter(REFLVL_BTN);
        }
        // DPRINTF("Set REFLVL to ON "); DPRINT(std_btn[REFLVL_BTN].enabled);
    }

    if (toggle == 0 || toggle == -1)
    {
        std_btn[REFLVL_BTN].enabled = OFF;
        MeterInUse                  = false;
        if (toggle != -1)
            clearMeter();
        // DPRINTF("Set REFLVL to OFF "); DPRINT(std_btn[REFLVL_BTN].enabled);
    }

    displayRefLevel();
    // DPRINTF(" and Ref Level is "); DPRINTLN(Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_floor);
}

// Ref Level adjust
// Pass the desired new absolute value.  It will be limited to allowable range of -110 to -220
COLD void RefLevel(int8_t newval)
{
    bandmem[curr_band].sp_ref_lvl += newval;
    if (bandmem[curr_band].sp_ref_lvl > 50)
        bandmem[curr_band].sp_ref_lvl = 50;
    if (bandmem[curr_band].sp_ref_lvl < -50)
        bandmem[curr_band].sp_ref_lvl = -50;
#ifndef BYPASS_SPECTRUM_MODULE
    Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_floor = bandmem[curr_band].sp_ref_lvl;
#endif
    displayRefLevel();
    // DPRINTF("Set Reference Level to "); DPRINTLN(Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_floor);
}

// Notch button
COLD void Notch()
{
    if (user_settings[user_Profile].notch == ON)
    {
        user_settings[user_Profile].notch = OFF;
        //LMS_Notch.enable(false);
    }
    else if (user_settings[user_Profile].notch == OFF)
    {
        user_settings[user_Profile].notch = ON;
        //LMS_Notch.enable(true);
        //DPRINTLN(LMS_Notch.initializeLMS(2, 32, 4)); // <== Modify to suit  2=Notch 1=Denoise
        //LMS_Notch.setParameters(0.05f, 0.999f);      // (float _beta, float _decay);
    }

    displayNotch();
    // DPRINT("Set Notch to "); DPRINTLN(user_settings[user_Profile].notch);
}

// BAND UP button
COLD void BandUp()
{
    changeBands(1);
    displayBandUp();
    // DPRINTF("Set Band UP to "); DPRINTLN(bandmem[curr_band].band_num,DEC);
}

// BAND DOWN button
COLD void BandDn()
{
    // DPRINTLN("BAND DN");
    changeBands(-1);
    displayBandDn();
    // DPRINTF("Set Band DN to "); DPRINTLN(bandmem[curr_band].band_num,DEC);
}

// BAND button
// Bandstack will copy in one of the alternative saved VFOA values and cycle throgh them.  VFO_A_last is always the last active value. Last_1 the previous, and last_2 previous to that.
COLD void Band(uint8_t new_band)
{
    if (std_btn[BAND_BTN].enabled == ON)
    {
        if (popup && new_band != 255)
        {
            if (curr_band == new_band) // already on this band so new request must be for bandstack, cycle through, saving changes each time
            {
                DPRINT("Previous VFO A on Band ");
                DPRINTLN(curr_band);
                uint64_t temp_vfo_last;
                uint8_t temp_mode_last;

                temp_vfo_last                   = bandmem[curr_band].vfo_A_last; // Save  current freq and mode
                temp_mode_last                  = bandmem[curr_band].mode_A;
                bandmem[curr_band].vfo_A_last   = bandmem[curr_band].vfo_A_last_1; // shuffle previous up to curent
                bandmem[curr_band].mode_A       = bandmem[curr_band].mode_A_1;
                bandmem[curr_band].vfo_A_last_1 = bandmem[curr_band].vfo_A_last_2; // shuffle more
                bandmem[curr_band].mode_A_1     = bandmem[curr_band].mode_A_2;
                bandmem[curr_band].vfo_A_last_2 = temp_vfo_last; // let changeBands compute new band based on VFO frequency
                bandmem[curr_band].mode_A_2     = temp_mode_last;
                VFOA                            = bandmem[curr_band].vfo_A_last; // store in the Active VFO register
            }
            else
            {
                DPRINT("Last VFO A on Band ");
                DPRINTLN(new_band);
                VFOA = bandmem[new_band].vfo_A_last; // let changeBands compute new band based on VFO frequency
            }
            changeBands(0);
        }
        std_btn[BAND_BTN].enabled = OFF;
        displayBand_Menu(0); // Exit window
    }
    else
    {
        std_btn[BAND_BTN].enabled = ON;
        displayBand_Menu(1); // Init window
    }
    displayBand();
    displayMode();
    displayFilter();
    // displayRefresh();
    displayFreq(); // show freq on display
    // DPRINTF("Set Band to "); DPRINTLN(bandmem[curr_band].band_num,DEC);
}

// DISPLAY button
COLD void Display()
{
#ifndef BYPASS_SPECTRUM_MODULE
    if (Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_dot_bar_mode)
    {
        display_state                                                          = 0;
        Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_dot_bar_mode = 0;
    }
    else
    {
        display_state                                                          = 1;
        Sp_Parms_Def[user_settings[user_Profile].sp_preset].spect_dot_bar_mode = 1;
    }
    drawSpectrumFrame(user_settings[user_Profile].sp_preset);
#endif
    // popup = 1;
    // pop_win_up(1);
    displayDisplay();
    // DPRINTF("Set Display Button to "); DPRINTLN(display_state);
}

COLD void TouchTune(int16_t touch_Freq)
{
    if (popup == 1) return; // skip if menu window is active
    selectFrequency(0);
    displayFreq();
}

COLD void selectStep(uint8_t fndx)
{
    if (fndx <= 1)
    {
        fndx = 0;
    }

    if (fndx >= TS_STEPS - 1)
    {
        fndx = TS_STEPS - 1;
    }
    //  remove the int fndx arg if using a global fndx
    bandmem[curr_band].tune_step = fndx;
    displayRate();
}

COLD void selectAgc(uint8_t andx)
{
    //struct AGC* pAGC = &agc_set[andx];

    if (andx >= AGC_FAST)
        andx = AGC_SLOW; // Cycle around

    if (andx <= AGC_SLOW)
        andx = AGC_FAST; // Cycle around

    bandmem[curr_band].agc_mode = andx;

    displayAgc();
}

// Turns meter off
void clearMeter(void)
{
    DPRINTLN("Turn OFF meter");
    MeterInUse = false;
    // blank out text line from overruns
    draw_2_state_Button(SMETER_BTN, &std_btn[SMETER_BTN].show);
    set_MF_Service(encoder_list[default_MF_slot].default_MF_client); // will turn off the button, if any, and set the default as new owner.
    MF_default_is_active = true;
}

// Turns meter on and assigns function to the new MF focus
void setMeter(uint8_t id)
{
    if (MF_client != id)
    {
        // DPRINTLN("Turn ON meter");
        set_MF_Service(id); // reset encoder counter and set up for next read if any until another functionm takes ownership
        MF_default_is_active = false;
    }
}

// Toggle the assigned function on an encoder shaft.
void setEncoderMode(uint8_t role)
{
    if (MF_client != role) // Probably don't do this if assigned to a multifunction knob sicne it is temporary
    {
        DPRINT("Encoder Switch ID = ");
        DPRINTLN(role);
        int slot;
        // find enabled aux encoders in order of slot assignment and copy to local var for processing in a loop, skip VFO slot
        for (slot = 1; slot < NUM_AUX_ENCODERS; slot++)
        {
            #ifdef DEBUG
                uint8_t _type, _enabled, _id, _mfenc, _roleA, _a_active, _roleB, _tap, _press;

                _type     = encoder_list[slot].type;
                _id       = encoder_list[slot].id;
                _enabled  = encoder_list[slot].enabled;
                _mfenc    = encoder_list[slot].default_MF_client;
                _roleA    = encoder_list[slot].role_A;
                _a_active = encoder_list[slot].a_active;
                _roleB    = encoder_list[slot].role_B;
                _tap      = encoder_list[slot].tap;
                _press    = encoder_list[slot].press;

                DEBUG_PRINTF("Slot#=%1d type=%1d id=0x%1d enabled=%1d MFENC?=%2d RoleA=%2d, A_active=%1d RoleB=%2d TAP=%2d PRESS=%2d\n",
                            slot, _type, _id, _enabled, _mfenc, _roleA, _a_active, _roleB, _tap, _press);
            #else
                uint8_t _tap;
                _tap = encoder_list[slot].tap;
            #endif

            // when switch is tapped or pressed, toggle which function is assigned to the encoder rotation
            // Uses the KEYWORD TOGGLE in thr encoder_list table
            if (_tap == role)
            {
                switch (_tap)
                {
                    case SW1_BTN:
                    case SW2_BTN:
                    case SW3_BTN:
                    case SW4_BTN:
                    case SW5_BTN:
                    case SW6_BTN:
                        DPRINT("Toggle Active Control ");
                        DPRINTLN(role);
                        encoder_list[slot].a_active ^= 1;
                        update_icon_outline();
                        // if was MF knob, turn on the normal meter asap to return to normal use fast.
                        if (MF_client != MFTUNE && !encoder_list[slot].a_active)
                        {
                            MF_Timeout.reset();
                            clearMeter();
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

// Takes a 0 to 100 input, converts to the appropriate hardware steps such as 0-31dB in 1 dB steps
// Code is for the PE4302 digital step attenuator
// Can clone and modify this for other hardware, call it from teh ATTN() function
COLD void digital_step_attenuator_PE4302(int16_t _attn)
{
    #ifndef PE4302
        DPRINTLN(F("PE4302 digital step attenuator not configured, skipping"));
        return; // Wrong hardware if not PE4302
    #else

    const uint8_t atten_size_31 = 62;  // 62 steps  31dB in 0.5 dB steps. Use 31 for 1dB steps
  
    char atten_str[8]  = {'\0'};
    char atten_data[8] = {'\0'};
    uint8_t i;
    int16_t attn;

    // scale 0 to 100% to size of attenuator hardware.  Assuming 0 to 31dB here.
    attn = round(((_atten * atten_size_31 / 100) - 0.5)); // round down to get 0-31 range

    if (attn >= atten_size_31) // will crash is exceed 0 to 31!
        attn = atten_size_31;
    if (attn < 0)
        attn = 0;

    //DPRINT("digital step converted = ");DPRINTLN(attn);

    // comment out for 0.5dB steps
    //attn *= 2; // shift the value x2 so the LSB controls the 1dB step.

    /* Convert to 8 bits of  0 and 1 format */
    itoa(attn, atten_str, 2);
    //DPRINTLN(atten_str);   // should be 6 bits of binary in the range of 0 to 64
    
    // Convert to 6 bits of  0 and 1 format 
    // pad with leading 0s as needed.  6 bits for the PE4302 + '\0' at end for 7 bytes
    //snprintf(atten_data, 7, "%06s", atten_str);   // works but get compiler warning for 0 and string together    
    for (i = 0; (i < 6 - strlen(atten_str)); i++)
    {
        atten_data[i] = '0';
    }
    strncat(atten_data, atten_str, strlen(atten_str));
    //DPRINTLN(atten_data);   // should be 6 bits of binary in the range of 0 to 64

    //  LE = 0 to allow writing data into shift register
    digitalWrite(Atten_LE, (uint8_t)OFF);
    digitalWrite(Atten_DATA, (uint8_t)OFF);
    digitalWrite(Atten_CLK, (uint8_t)OFF);
    delayMicroseconds(10);
 
    //  Now loop for 6 bits, set data on Data pin and toggle Clock pin.
    //    Start with the MSB first so start at the left end of the string
    for (i = 0; i < 6; i++)
    {
        // convert ascii 0 or 1 to a decimal 0 or 1
            digitalWrite(Atten_DATA, (uint8_t) atten_data[i]-48);
            delayMicroseconds(10);
            digitalWrite(Atten_CLK,  (uint8_t) ON);
            delayMicroseconds(10);
            digitalWrite(Atten_CLK,  (uint8_t) OFF); 
            delayMicroseconds(10);
        }
        //  Toggle LE pin to latch the data and set the new attenuation value in the hardware
        digitalWrite(Atten_LE, (uint8_t) ON);
        delayMicroseconds(10);
        digitalWrite(Atten_LE, (uint8_t) OFF);
    #endif
}

//
// Changes to the correct band settings for the new target frequency.
// If the new frequency is below or above the band limits it returns 0 else returns the new frequency
//
//#define DBG_BAND
HOT uint64_t find_new_band(uint64_t new_frequency, uint8_t &_curr_band)
{
    int i;

    #ifdef DBG_BAND
        DPRINTF("find_band(): New Frequency requested = "); DPRINTLN(new_frequency);
        DPRINTF("find_band(): New Band requested = "); DPRINTLN(_curr_band);
    #endif

    for (i=BANDS-1; i>=0; i--)    // start at the top and look for first band that VFOA fits under bandmem[i].edge_upper
    {
        #ifdef DBG_BAND
            DPRINTF("find_band(): Edge_Lower Search = "); DPRINTLN(bandmem[i].edge_lower);
        #endif
        if (new_frequency >= bandmem[i].edge_lower && new_frequency <= bandmem[i].edge_upper) // found a band lower than new_frequency so search has ended
        {
            #ifdef DBG_BAND
                DPRINTF("find_band(): Edge_Lower = "); DPRINTLN(bandmem[i].edge_lower);
            #endif
            _curr_band = bandmem[i].band_num;
            #ifdef DBG_BAND
                DPRINTF("find_band(): New Band = "); DPRINTLN(_curr_band);
            #endif
            //  Calculate frequency difference between the designated xvtr IF band's lower edge and the current VFO band's lower edge (the LO frequency).
            //if (bandmem[*_curr_band].xvtr_IF)
            //    xvtr_offset = bandmem[*_curr_band].edge_lower - bandmem[bandmem[*_curr_band].xvtr_IF].edge_lower; // if band is 144 then PLL will be set to VFOA-xvtr_offset
            //else
            //    xvtr_offset = 0;
            if (bandmem[_curr_band].bandmap_en) // filter out disabled bands
                return new_frequency;
            else
            {
                DPRINTF("find_band(): Disabled band requested "); DPRINTLN(bandmem[_curr_band].band_name);
                return 0;
            }
        }
    }
    //#ifdef DBG_BAND
       // DPRINTLNF("MAIN: Invalid Frequency Requested");
    //#endif
    return 0; // 0 means frequency was not found in the table
}

// Used to request extended mode from radio mode so we can get teh DATa on/off status.  Radio does not tell us when the DATA mode is changed
COLD uint8_t get_Mode_from_Radio(void) 
{
    CIVresult_t CIVresultL_mode;

    CIVresultL_mode = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(cmd_List[CIV_C_F26A].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("get_Mode_from_Radio: retVal of Mode cmd writeMsg: "); DPRINTLN(retValStr[CIVresultL_mode.retVal]);
    Check_radio();
    return CIVresultL_mode.retVal;
}

// Used to set remote's mode to what heh radio reported, usually by getmode()
COLD void set_Mode_from_Radio(uint8_t mndx)   // Change Mode of the current active VFO by increment delta.
{
    if ((curr_band < BAND1296) && (mndx > MODES_NUM-3))  // DD and ATV not avaiable on bands < 1296
    {
        DPRINTF("set_Mode_from_Radio: request mode mode not available on bands < 1296: ");  DPRINTLN(modeList[mndx].mode_label);  
        mndx = MODES_NUM -3;    // go to start of list
    }

    DPRINT("set_Mode_from_Radio: Set mode to "); DPRINTLN(modeList[mndx].mode_label);  	
    bandmem[curr_band].mode_A = mndx; // get current mode table index
    
    displayDATA(); // update display
    displayMode();
    displayFilter();
}

//  Used when changing bands from the remote side.  
COLD void send_Mode_to_Radio(uint8_t mndx) 
{
    CIVresult_t CIVresultL;
    uint8_t data_str[6] = {};

    //DPRINT("send_Mode_to_Radio: mode index: "); DPRINTLN(mndx);  	

    if ((curr_band < BAND1296) && (mndx > MODES_NUM-3))  // DD and ATV not avaiable on bands < 1296
    {
        DPRINTF("send_Mode_to_Radio: request mode mode not available on bands < 1296: ");  DPRINTLN(modeList[mndx].mode_label);  
        mndx = MODES_NUM -3;    // go to start of list
    }
    
    if (modeList[mndx].data == 1)  // selected xxx-D modes needs DATA enabled on radio or not
        radio_data = bandmem[curr_band].data_A = 1;  // set DATA on or off 
    else 
        radio_data = bandmem[curr_band].data_A = 0;  // set DATA on or off 

    radio_mode = modeList[mndx].mode_num;
    radio_filter = bandmem[curr_band].filter_A = modeList[mndx].Width;
    
    //    bandmem[curr_band].data_A = 1;  // set DATA on or off 

    // Populate the datafield
    data_str[0] = 3;  // send the mode values
    data_str[1] = radio_mode;  // send the mode values
    data_str[2] = radio_data;  // set DATA on or off 
    data_str[3] = radio_filter;  // Set filter
    
    DPRINTF("send_Mode_to_Radio: Mode: "); DPRINT(modeList[mndx].mode_label); DPRINTF("  Filter: "); DPRINT(filter[radio_filter].Filter_name); DPRINTF("    Data: "); DPRINTLN(radio_data);    

    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_F26A].cmdData), reinterpret_cast<const uint8_t*>(data_str), CIV_wChk);
    //CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_MOD_READ].cmdData), reinterpret_cast<const uint8_t*>(data_str), CIV_wChk);
    //CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, data_str, CIV_D_NIX, CIV_wChk);
    //while (CIVresultL_vfo.retVal > CIV_OK_DAV)
    //{
    //    delay(40);
    //    CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, data_str, CIV_D_NIX, CIV_wChk);
    //    DPRINTF("send_Mode_to_Radio: delay loop ret = "); DPRINTLN(retValStr[CIVresultL_vfo.retVal]);  
    //}
    
    //DPRINTF("send_Mode_to_Radio: retVal of Mode cmd writeMsg: "); DPRINTLN(retValStr[CIVresultL.retVal]);
    
    if (CIVresultL.retVal == CIV_OK)
    {
        DPRINT("send_Mode_to_Radio: Set mode to "); DPRINTLN(modeList[mndx].mode_label);  	
        bandmem[curr_band].mode_A = mndx; // get current mode table index
        Check_radio();
        displayMode();
        displayFilter();
        displayDATA();
    }
    return;
}

//  Used to read the Band stack register of choice
//  Could be useful to read mode, filter, and other settings for each band on controller startup
COLD uint8_t read_BSTACK_from_Radio(uint8_t band, uint8_t reg)   // Ask the radio for band and register contents
{
    CIVresult_t CIVresultL_vfo;

    //DPRINTF("read_BSTACK_from_Radio: Band stack band and register contents requested: "); DPRINT(band); DPRINTF(" "); DPRINTLN(reg);

    uint8_t data_str[3] = {};
    data_str[0] = 2;  // send the mode values
    data_str[1] = band;  // send the mode values
    data_str[2] = reg;  // send the mode values

    CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_BSTACK].cmdData), reinterpret_cast<const uint8_t*>(data_str), CIV_wChk);
    //DPRINTF("read_BSTACK_from_Radio: retVal of BSTACK writeMsg: "); DPRINTLN(retValStr[CIVresultL_vfo.retVal]);
    delay(20);
    Check_radio();
    if (CIVresultL_vfo.retVal == CIV_OK)
    {
        //DPRINT("read_BSTACK_from_Radio: Value is "); DPRINTLN(CIVresultL_vfo.value);  	
        return 0;
    }
    return 1;
}

// Used to request frequency from radio
COLD uint64_t get_Freq_from_Radio(void)   // Change Mode of the current active VFO by increment delta.
{
    CIVresult_t CIVresultL;

    CIVresultL = civ.writeMsg(CIV_ADDR_905, cmd_List[CIV_C_F_READ].cmdData, CIV_D_NIX, CIV_wChk);  // kick off freq request to update from radio
    DPRINTF("get_Freq_from_Radio: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
    //return CIVresultL.value;  // return 
    delay(20);
    Check_radio();
    return 1;  // return 1 for freq success, 0 for nothing, or any other value we are not looking for
}

// Used to request RX TX status from radio
COLD uint8_t get_RXTX_from_Radio(void)   // Change Mode of the current active VFO by increment delta.
{
    CIVresult_t CIVresultL;

    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_TX].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("get_RXTX_from_Radio: retVal of RX TX: "); DPRINTLN(retValStr[CIVresultL.value]);
    Check_radio();
    return CIVresultL.value;
}

// Used to request position adn time from radio
COLD uint8_t get_MY_POSITION_from_Radio(void)
{
    CIVresult_t CIVresultL;

    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_UTC_OFFSET].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("get_MY_POSITION_from_Radio: retVal of UTC Offset: "); DPRINTLN(retValStr[CIVresultL.value]);
    delay(60);
    check_CIV(millis());  // give time to respond -  Msg_type 3 is bstack results
    
    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_MY_POSIT_READ].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("get_MY_POSITION_from_Radio: retVal of MY POS: "); DPRINTLN(retValStr[CIVresultL.value]);
    delay(60);
    check_CIV(millis());  // give time to respond -  Msg_type 3 is bstack results

    return CIVresultL.value;
}

// Used to request status from radio
COLD uint8_t get_Preamp_from_Radio(void)
{
    CIVresult_t CIVresultL;

    if (curr_band < BAND2400)  // IC905 does not have Attn or Preamp on bands > 1296
    {
        CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_PREAMP_READ].cmdData), CIV_D_NIX, CIV_wChk);
        DPRINTF("get_PreAmp_from_Radio: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
        delay(20);
        Check_radio();
        return CIVresultL.value;
    }
    else
    {
        DPRINTLNF("get_Preamp_from_Radio: skipping for bands > 1296");
    }
    return 0;
}

// Used to request status from radio
COLD uint8_t get_Attn_from_Radio(void)
{
    CIVresult_t CIVresultL;

    if (curr_band < BAND2400)  // IC905 does not have Attn or Preamp on bands > 1296
    {
        CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_ATTN_READ].cmdData), CIV_D_NIX, CIV_wChk);
        DPRINTF("get_Attn_from_Radio: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
        delay(20);
        Check_radio();
        return CIVresultL.value;
    }
    else
    {
        DPRINTLNF("get_Attn_from_Radio: skipping for bands > 1296");
    }
    return 0;
}

// Used to request status from radio
COLD uint8_t get_AGC_from_Radio(void)
{
    CIVresult_t CIVresultL;

    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_AGC_READ].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("get_AGC_from_Radio: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
    delay(20);
    Check_radio();
    return CIVresultL.value;
}

// Used to request status from radio - certain bands allow only AGC FAST, that is taken care by the calling function
COLD uint8_t send_AGC_to_Radio(void)
{
    CIVresult_t CIVresultL;
    
    cmd_List[CIV_C_AGC_FAST].cmdData[3] = bandmem[curr_band].agc_mode;
    CIVresultL = civ.writeMsg(CIV_ADDR_905, reinterpret_cast<const uint8_t*>(&cmd_List[CIV_C_AGC_FAST].cmdData), CIV_D_NIX, CIV_wChk);
    DPRINTF("send_AGC_to_Radio: retVal: "); DPRINTLN(retValStr[CIVresultL.value]);
    delay(20);
    Check_radio();
    return CIVresultL.value;
}

// Very basic - oupuits a set pattern for each band.  Follows the ELecraft K3 patther for comnbined HF and VHF used for transverters and antenna swicthing
// This may control a external band decoder that accept wired inputs.  Other decoder outpout can be serial or ethernet
void Band_Decode_Output(uint8_t band)
{
    // Convert frequency band to a parallel wire GPIO putput pattern.
    // On a Elecraft K3 this is equivalent to the HF-TRN mode.  Digout is used in combo with Band Decode BCD 0-3 pins.  
    // The pattern 0xYYXX where YY is 01 for VHF+ band group and 00 for HF band group.  XX is the band identifier witin each HF and VHF group.
    // Edit this to be anyting you want.
    // Set your desired patterns in RAdioConfig.h
    // ToDo: Eventually create a local UI screen to edit and monitor pin states

    DPRINTF("Band_Decode_Output: Band: "); DPRINTLN(band);

    switch (band)
    {
       case  BAND160M : GPIO_Out(DECODE_BAND160M); break;   //160M 
       case  BAND80M  : GPIO_Out(DECODE_BAND80M);  break;   //80M
       case  BAND60M  : GPIO_Out(DECODE_BAND60M);  break;   //60M
       case  BAND40M  : GPIO_Out(DECODE_BAND40M);  break;   //40M
       case  BAND30M  : GPIO_Out(DECODE_BAND30M);  break;   //30M
       case  BAND20M  : GPIO_Out(DECODE_BAND20M);  break;   //20M
       case  BAND17M  : GPIO_Out(DECODE_BAND17M);  break;   //17M      
       case  BAND15M  : GPIO_Out(DECODE_BAND15M);  break;   //15M
       case  BAND12M  : GPIO_Out(DECODE_BAND12M);  break;   //12M
       case  BAND10M  : GPIO_Out(DECODE_BAND10M);  break;   //10M
       case  BAND6M   : GPIO_Out(DECODE_BAND6M);   break;   //6M
        //case BAND70   : GPIO_Out(0x01); break;   //6M
       case  BAND144  : GPIO_Out(DECODE_BAND144);  break;   //2M
       case  BAND222  : GPIO_Out(DECODE_BAND222);  break;   //222
       case  BAND432  : GPIO_Out(DECODE_BAND432);  break;   //432
       case  BAND902  : GPIO_Out(DECODE_BAND902);  break;   //902
       case  BAND1296 : GPIO_Out(DECODE_BAND1296); break;   //1296
       case  BAND2400 : GPIO_Out(DECODE_BAND2400); break;   //2400
       case  BAND3400 : GPIO_Out(DECODE_BAND3400); break;   //3400
       case  BAND5760 : GPIO_Out(DECODE_BAND5760); break;   //5760M
       case  BAND10G  : GPIO_Out(DECODE_BAND10G);  break;   //10.368.1G
       case  BAND24G  : GPIO_Out(DECODE_BAND24G);  break;   //24.192G
       case  BAND47G  : GPIO_Out(DECODE_BAND47G);  break;   //47.1G
       case  BAND76G  : GPIO_Out(DECODE_BAND76G);  break;   //76.1G
       case  BAND122G : GPIO_Out(DECODE_BAND122G); break;   //122G
    }
}

void GPIO_Out(uint8_t pattern)
{
    DPRINTF("GPIO_Out: pattern:  DEC "); DPRINT(pattern);
    DPRINTF("  HEX "); DPRINT(pattern, HEX);
    DPRINTF("  Binary "); DPRINTLN(pattern, BIN);
    
    // mask each bit and apply the 1 or 0 to the assigned pin
    digitalWrite(BAND_DECODE_OUTPUT_PIN_0, pattern & 0x01);  // bit 0
    digitalWrite(BAND_DECODE_OUTPUT_PIN_1, pattern & 0x02);  // bit 1
    digitalWrite(BAND_DECODE_OUTPUT_PIN_2, pattern & 0x04);  // bit 2
    digitalWrite(BAND_DECODE_OUTPUT_PIN_3, pattern & 0x08);  // bit 3
    digitalWrite(BAND_DECODE_OUTPUT_PIN_4, pattern & 0x10);  // bit 4
    digitalWrite(BAND_DECODE_OUTPUT_PIN_5, pattern & 0x20);  // bit 5
    digitalWrite(BAND_DECODE_OUTPUT_PIN_6, pattern & 0x40);  // bit 6
    digitalWrite(BAND_DECODE_OUTPUT_PIN_7, pattern & 0x80);  // bit 7
}

void Decoder_GPIO_Pin_Setup(void)
{
    // using 8 bits since the ouput pattern is 1 byte.  Can use thenm any way you want. 
    // The pins used here are defined in RadioConfig.  The one GPIO_SWx_PIN were designated as hardware switches in the Teensy SDR 
    // If using the Teensy SDR motherboard and you have physical switch hardware on any of these then you need to pick alernate pins.
    // Most pins are alrewady goiven a #define bname in RadioCOnfig, substitute the right ones in here.  Make sure they are free.
    pinMode(BAND_DECODE_OUTPUT_PIN_0, OUTPUT);  // bit 0
    pinMode(BAND_DECODE_OUTPUT_PIN_1, OUTPUT);  // bit 1
    pinMode(BAND_DECODE_OUTPUT_PIN_2, OUTPUT);  // bit 2
    pinMode(BAND_DECODE_OUTPUT_PIN_3, OUTPUT);  // bit 3
    pinMode(BAND_DECODE_OUTPUT_PIN_4, OUTPUT);  // bit 4
    pinMode(BAND_DECODE_OUTPUT_PIN_5, OUTPUT);  // bit 5
    pinMode(BAND_DECODE_OUTPUT_PIN_6, OUTPUT);  // bit 6
    pinMode(BAND_DECODE_OUTPUT_PIN_7, OUTPUT);  // bit 7
    
    DPRINTLNF("Decoder_GPIO_Pin_Setup: Pin Mode Setup complete");
}