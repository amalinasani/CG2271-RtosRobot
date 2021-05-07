#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"
#include "cmsis_os2.h"
#include "system_MKL25Z4.h"
#include "musical_notes.h"
#include  CMSIS_device_header

#define PTB3_BUZZER 3
#define FREQ_2_MOD(x) (375000/x)

/*
 * Function:  initBuzzer
 * --------------------
 * Initialises PWM for Port B Pin 3 GPIO pin. 
 *
 */
void initBuzzer(void) {
	// Enable clock for PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; 
	
	// Configure Mode 3 (Alternative 3 - TPM2_CH1)
	PORTB->PCR[PTB3_BUZZER] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_BUZZER] |= PORT_PCR_MUX(3); 
	
	// Enable clock for TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	TPM2->MOD = 7500;
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

/*
 * Function:  playNote
 * --------------------
 * Plays the specific note for a set duration.
 * 
 * tone: 			The integer value of the tone to be played
 * duration:	Duration in which the tone is played for in ms
 *
 */
void playNote(int note, 
							int duration) {
	TPM2->MOD = FREQ_2_MOD(note);
	TPM2_C1V = (FREQ_2_MOD(note)) / 2;
	osDelay(duration);
}

/*
 * Function:  playBtSong
 * --------------------
 * Plays the melody for when bluetooth connection is established. (Nokia ringtone) 
 *
 */
void playBtSong(void) {
	int notes = sizeof(bt_connect_melody) / sizeof(bt_connect_melody[0]) / 2;
	int wholenote = (60000 * 4) / 280;
	int divider = 0, noteDuration = 0;
	
  // Iterate over the notes of the melody. 
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
		// Calculates the duration of each note
    divider = bt_connect_melody[thisNote + 1];
    noteDuration = (wholenote) / divider;

    // Play the note for 90% of the duration, leaving 10% as a pause.
    playNote (bt_connect_melody[thisNote], 
							noteDuration * 0.9);

    // Wait for the specified duration before playing the next note.
    osDelay(noteDuration);
  }
}

/*
 * Function:  playEndSong
 * --------------------
 * Plays the melody for the victory song. ("Take on me" by a-ha) 
 *
 */
void playEndSong(void) {
	int notes = sizeof(end_melody) / sizeof(end_melody[0]) / 2;
	int wholenote = (60000 * 4) / 220;
	int divider = 0, noteDuration = 0;
	
  // Iterate over the notes of the melody. 
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    // Calculates the duration of each note.
    divider = end_melody[thisNote + 1];
    noteDuration = (wholenote) / divider;

    // Play the note for 90% of the duration, leaving 10% as a pause.
    playNote (end_melody[thisNote], 
							noteDuration * 0.9);

    // Wait for the specified duration before playing the next note.
    osDelay(noteDuration);
  }
}

/*
 * Function:  playSong
 * --------------------
 * Plays the melody for the entire run. ("Mii Channel Music" by Kazumi Totaka) 
 *
 */
void playSong(int thisNote) {
	// Duration of a whole note in ms
	int wholenote = (60000 * 4) / 180;
	int divider = 0, 
			noteDuration = 0;	

	divider = miiMelody[thisNote + 1];
	noteDuration = (wholenote) / divider;

	// Play the note for 90% of the duration, leaving 10% as a pause.
	playNote (miiMelody[thisNote], 
						noteDuration * 0.9);

	// Wait for the specied duration before playing the next note.
	osDelay(noteDuration);
}

/*
 * Function:  stopSong
 * --------------------
 * Stops currently playing melody. 
 *
 */
void stopSong(void) {
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
}
