Radar Control Program Name:
==========================
interleavesound_stereo

Control Program ID (CPID):
=========================
191 (ChA) / 197 (ChB)

Parameters:
==========
nbeams: 16
intt: 3 s
scan: 1 min
ngates: 70
frang: 180 km
rsep: 45 km

Description:
===========
interleavesound_stereo is a variant on the stereo_interleave
and interleavesound radar control programs. This mode performs
a scan in a nonsequential manner by "interleaving" the beam
number, e.g. (0-4-8-12)-(2-6-10-14)-(1-5-9-13)-(3-7-11-15),
on Channel A. On Channel B, it performs scans through a set of
up to 10 frequencies and through all beams. Note that unlike
normalsound, this information is not used to adjust the radar
operating frequency in real-time.

The control program requires radar-specific frequency bands
for Channel B defined in the "u_init_freq_bands()" function.
Also, a list of radar-specific frequency band indices must
be defined within "interleavesound_stereo.c"; currently only
default values for HAN, PYK, and HKW are included.

The sounding data are written to *.snd files in the SD_SND_PATH
directory. If this environment variable is not set, the control
program will attempt to write the sounding data to the "/data/snd"
directory. If this directory does not exist, no sounding data will
be written. The 2-hr sounding files contain a reduced set of
radar operating parameters and fitted values (e.g., velocity,
power, spectral width, phi0) in dmap-format.

The "-nosnd" option can be used to disable the output *.snd files
if necessary (the frequency sounding data will always be stored on
Channel B of the output rawacf files).

The "-katscan" option can be used to enable the 8-pulse sequence
rather than the 7-pulse sequence used by default at Stereo radars.

Source:
======
E.G. Thomas (20201012)
