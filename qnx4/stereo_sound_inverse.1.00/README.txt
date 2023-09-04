Radar Control Program Name:
==========================
stereo_sound_inverse

Control Program ID (CPID):
=========================
20155 / 20157 (ChA)
155 / 157 (ChB)

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
stereo_sound_inverse is a variant on the stereo_sound radar
control program. This mode performs a normal 1- or 2-min scan in
a sequential manner on Channel B at a fixed frequency band. On
Channel A, it performs scans through a set of different frequencies
on all beams. Note that unlike previous versions of normalsound,
this information is not used to adjust the radar operating frequency
in real-time. Also note that this control program does not produce
*.snd files.

The control program requires radar-specific frequency bands for
Channel A defined in the "u_init_freq_bands()" function. Also,
a list of radar-specific frequency band indices must be defined
within "stereo_sound_inverse.c"; currently only default values for
Hokkaido West (HKW) and Longyearbyen (LYR) are included.

Source:
======
E.G. Thomas (20230904)
