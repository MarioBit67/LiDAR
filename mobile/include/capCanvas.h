#ifndef CAPCANVAS_H
#define CAPCANVAS_H
#include "winTypes.h"
#include "capPort.h"

/* Software drawing on a TSurface - common to every platform, so the screen looks the same everywhere.
 * Colors are 0xAABBGGRR (memory order R, G, B, A); alpha blends over what is already there. */

DWORD canvasRGBA(int r, int g, int b, int a);
void  canvasFillRect(TSurface &s, int x, int y, int w, int h, DWORD rgba);
void  canvasFillCircle(TSurface &s, int cx, int cy, int radius, DWORD rgba);
void  canvasRing(TSurface &s, int cx, int cy, int radius, int thickness, DWORD rgba);
void  canvasLine(TSurface &s, int x0, int y0, int x1, int y1, int thickness, DWORD rgba); // opaque colors: dots overlap
void  canvasFillQuad(TSurface &s, const int *xs, const int *ys, DWORD rgba); // convex, 4 corners in order

// Portrait RGB preview (pixels in canvas order) scaled to fill the rect, center-cropped
void canvasBlit(TSurface &s, int x, int y, int w, int h, const DWORD *img, int iw, int ih);

#endif // CAPCANVAS_H
