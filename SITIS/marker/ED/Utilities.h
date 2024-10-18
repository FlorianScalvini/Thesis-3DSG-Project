//
// From https://github.com/bbenligiray/stag
// 

#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "EdgeMap.h"



void StdRGB2LabOne(unsigned char r, unsigned char g, unsigned char bl, double *L, double *a, double *b);


void DumpGradImage(char *file, short *gradImg, int width, int height);
void DumpGradImage(char *file, short *gradImg, int width, int height, int thresh);

void DumpEdgeSegments(char *file, EdgeMap *map);


void ColorEdgeSegments(EdgeMap *map, unsigned char *colorImg, unsigned char *srcImg=NULL);
void ColorEdgeSegments(char *file, EdgeMap *map, unsigned char *srcImg=NULL);

void ShowJointPoints(char *file, EdgeMap *map, unsigned char *jointPoints, unsigned char *srcImg=NULL);

struct DirectoryEntry {
  char filename[100];
};
int GetFilenamesInDirectory(char *dirname, DirectoryEntry *items);

/// Scales a given image
unsigned char *ScaleImage(unsigned char *srcImg, int width, int height, double scale, int *pw, int *ph);


///-------------------------------------------------------------------------------------------
/// Color generator
///
struct ColorGenerator {
  int color;

  ColorGenerator(){color = 0;}
  void getNextColor(int *r, int *g, int *b){
    switch (color){
      case 0: *r=255; *g=0; *b=0; break;
      case 1: *r=0; *g=255; *b=0; break;
      case 2: *r=0; *g=0; *b=255; break;
      case 3: *r=255; *g=255; *b=0; break;
      case 4: *r=0; *g=255; *b=255; break;
      case 5: *r=255; *g=0; *b=255; break;
      case 6: *r=255; *g=128; *b=0; break;
      case 7: *r=255; *g=0; *b=128; break;
      case 8: *r=128; *g=255; *b=0; break;
      case 9: *r=0; *g=255; *b=128; break;
      case 10: *r=128; *g=0; *b=255; break;
      case 11: *r=0; *g=128; *b=255; break;
      case 12: *r=0; *g=128; *b=128; break;
      case 13: *r=128; *g=0; *b=128; break;
      case 14: *r=128; *g=128; *b=0; break;
    } //end-switch

    color++;
    if (color>14) color=0;
  } //end-getNextColor
};

#endif