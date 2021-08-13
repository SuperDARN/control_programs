/* sndwrite.h
   ========== 
   Author: E.G.Thomas
*/

/*
 $Log: sndwrite.h,v $
 Revision 1.0  2020/06/25 egthomas
 Initial revision

*/

#ifndef _SNDWRITE_H
#define _SNDWRITE_H

int SndFwrite(FILE *fp,struct RadarParm *,struct FitData *);
int SndWrite(int fid,struct RadarParm *,struct FitData *);

#endif
