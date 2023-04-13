Radar Control Program Name:
==========================
stereo_sound

Control Program ID (CPID):
=========================
155 / 157 (ChA)
20155 / 20157 (ChB)

Parameters:
==========
nbeams: 16
intt: 7 s / 3 s
scan: 2 min / 1 min
ngates: 75+
frang: 180 km
rsep: 45 km

Description:
===========
stereo_sound is a variant on the stereoscan and normalsound radar
control programs. This mode performs a normal 1- or 2-min scan in
a sequential manner on Channel A at a fixed frequency band. On
Channel B, it performs scans through a set of different frequencies
on all beams. Note that unlike previous versions of normalsound,
this information is not used to adjust the radar operating frequency
in real-time. Also note that this control program does not produce
*.snd files.

The control program requires radar-specific frequency bands for
Channel B defined in the "u_init_freq_bands()" function. Also,
a list of radar-specific frequency band indices must be defined
within "stereo_sound.c"; currently only default values for Hokkaido
West (HKW) and Longyearbyen (LYR) are included.

Source:
======
E.G. Thomas (20230413)
