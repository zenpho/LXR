Brendan Clarke
Additions to LXR .34 firmware
LXR by Julian Schmidt (www.sonic-potions.com)

!!!BACK UP YOUR SD CARD!!!

Latest additions are at the top of the list.

16. Quick access to morph target parameters. When viewing a single parameter (click in and adjust with the encoder), press 'shift' to edit and view the parameter for the morph target.

15. One-shot LFO's! These are added as additional waveforms in the LFO settings. i.e. "si1", "tr1", etc are 1 shot versions of sine, triangle, and all the other normal lfo shapes. 
	- There is also a new LFO shape "exponential triangle" - "xtr" and "xt1" which are the exponential up followed immediately by exponential down. 
	-With a one-shot LFO selected, the 'offset' control instead sets a delay for the start of the LFO. It's a little weird because the delay gets scaled with 'rate' but it adds a lot of musical options. 
	- The 'noise' LFO holds a single random value on each retrigger. If there is a delay (offset) phase set, it holds the previous value during this time.
	- The 'rect' LFO is inverted in one-shot mode (normal phase is 0 first, then 1), so that it can be instantly on and then off after a time set by 'frq', with the 'offset' delay setting an off portion at the beginning if desired.

14. When the velocity of a step is set to '0', the step will not re-trigger envelopes. This lets you use steps as 'automation only' - a bit like 'trigless locks' on elektron kit.

13. Added a 'realign pattern' shortcut. pressing the pattern button of the currently viewed pattern in 'perf' mode will align all tracks to the master clock. This also resets any 'pattern rotation' set. Also, when holding shift in 'PERF' mode, you can press the voice buttons to quickly switch the active voice for pattern rotation. This is in addition to the multiple voice-unmute shortcut. shift+current pattern will also do the re-alignment.

12. added a global option to interpret Bank Change (CC0) messages incoming on the global channel as a command to load a 'Performance' data set instead of a drum kit. Bank changes on voice MIDI channels will still change that part individually.

11. 'Performance' and 'All' save types now save the morph target parameters in addition to the drum kit and pattern. The version number for this type of file has been incremented to 3. Previous versions will load with an empty morph target, but will be re-saved as the new type. 

10. Added pattern scaling option. This is accessed via a second page under 'click' (the transient voicing sub-page). Setting this increases track length by the shown multiplier, at the expense of step resolution. Note that this required changing some functions so that they do not use the step counter for DRUM1 as a reference. A dummy step counter was introduced instead for this (at the end of the step index array). It is unlikely, but possible this will cause problems with the trigger mod, but I have no way of testing this.

9. 'Load' menu now includes entries to load individual drum voices. This includes a number of load/save menu changes to deal with this:
	- The name and number of drum voices will reflect the drum kit the voice was derived from, even if this was changed via a MIDI bank change request.
	- The shortcut functions of the knobs has changed (knobs 1-4, from left to right):
	LOAD MENU
		- knob 1: change load type (kit, drum 1, pattern, etc)
		- knob 2: change preset number NB: auto-loads for all kit and voice types
		- knob 3: no function, except disables auto-load if turned
		- knob 4: disables auto-load, moves cursor between load type and 'ok' if it exists
	SAVE MENU
		- knob 1: change save type (same as for load menu)
		- knob 2: change cursor position (ie cycles through type, preset, alphanums, and 'ok')
		- knob 3: change cursor value
		- knob 4: for alphanumeric values, cycles through more characters
		note that the order of characters for the shortcut knobs has changed. Capital letters are all the way on the left of knob 3, numbers are in the middle of knob 3, and lower case is all the way to the left of knob 4.

8. MIDI channel select menus now have a '0' option that disables MIDI input.

7. Multiple voices can be set to respond (and record) note-ons on the global channel. When the currently active track has a 'note' assignment other than 'any', note-ons to the global channel will be matched to ALL tracks that have a note assignment other than 'any', rather than just the active one.

6. New global menu option to make the 'shift' button a toggle rather than momentary

5. Individual drum voices can be changed by sending a 'Bank MSB' (CC0) signal on their individual MIDI channels
	- bug fix - when 2 or more voices are stacked on a single channel, all will respond

4. Drumkit can be changed by sending a 'Bank MSB' (CC0) signal on the global MIDI channel

3. MIDI CC assignments are by voice channel rather than all on the global channel - see document in this folder for new assignments
	- this includes a morph assignment to Mod Wheel on the global channel
	- MIDI CC's also get recorded to automation slots (rather than updating voice parameters) when record is on
	- NRPN's are banished to the forbidden zone

2. New global menu option for instant pattern switching
	- when enabled, switching patterns with the buttons or a program change MIDI message happens on the next sub-step (keeping the current sequencer position) rather than at the end of the bar

1. Copy individual tracks between patterns.
	- select (view) the source pattern
	- press and hold 'copy'
	- press the source track (voice) button
	- press the destination pattern button

