
/**
 These two C-routines calculate the projection P of a point W on the ellipse,
 or on the surface of an Ellipsoid in 3D.
 The ellipse/ellipsoid is aligned with the principal axes (X, Y, Z),
 and has radii (radX, radY, radZ).

 Francois Nedelec. Copyright 2007-2017 EMBL.

 This code is Open Source covered by the GNU GPL v3.0 License
 */

#ifndef PROJECT_ELLIPSE_H
#define PROJECT_ELLIPSE_H

/// calculate `(pX, pY)`, the projection of `(wX, wY)` on the ellipse of axes
/// `radX, radY`
void projectEllipse(float* pX, float* pY, float wX, float wY, float radX,
                    float radY);

/// calculate `p`, the projection of a 3D point `w` on the ellipse of axes given
/// in `rad[]`
bool projectEllipsoid(float p[3], const float w[3], const float rad[3]);

#endif
