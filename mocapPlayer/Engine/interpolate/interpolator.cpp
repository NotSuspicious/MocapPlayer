#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"
#include "vector.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = BEZIER;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = QUATERNION;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON)
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  }
  else
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // Work with radians - don't modify input angles
    double anglesRad[3];
    for (int i = 0; i < 3; i++) {
        anglesRad[i] = angles[i] * M_PI / 180.0;
    }

    double cX = cos(anglesRad[0]);
    double sX = sin(anglesRad[0]);
    double cY = cos(anglesRad[1]);
    double sY = sin(anglesRad[1]);
    double cZ = cos(anglesRad[2]);
    double sZ = sin(anglesRad[2]);

    R[0] = cY * cZ;
    R[1] = sY * sX * cZ - cX * sZ;
    R[2] = sY * cX * cZ + sX * sZ;
    R[3] = cY * sZ;
    R[4] = sY * sX * sZ + cX * cZ;
    R[5] = sY * cX * sZ - sX * cZ;
    R[6] = -sY;
    R[7] = cY * sX;
    R[8] = cY * cX;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // Bezier control points for root position
        vector p1 = startPosture->root_pos;
        vector p2 = pInputMotion->GetPosture(startKeyframe + static_cast<int>(N*0.25))->root_pos;
        vector p3 = pInputMotion->GetPosture(startKeyframe + static_cast<int>(N*0.75))->root_pos;
        vector p4 = endPosture->root_pos;
        double controlPos[4][3] {
                {p1.x(), p1.y(), p1.z()},
                {p2.x(), p2.y(), p2.z()},
                {p3.x(), p3.y(), p3.z()},
                {p4.x(), p4.y(), p4.z()}
        };

        double basisControlPos[4][3];
        for(int i=0; i<4; i++)
            for(int j=0; j<3; j++)
            {
                basisControlPos[i][j] = 0;
                for(int k=0; k<4; k++)
                    basisControlPos[i][j] += BEZIER_BASIS[i*4 + k] * controlPos[k][j];
            }

        // Bezier control points for bone rotations
        double controlRot[MAX_BONES_IN_ASF_FILE][4][3];
        double basisControlRot[MAX_BONES_IN_ASF_FILE][4][3];
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            vector r1 = startPosture->bone_rotation[bone];
            vector r2 = pInputMotion->GetPosture(startKeyframe + static_cast<int>(N*0.25))->bone_rotation[bone];
            vector r3 = pInputMotion->GetPosture(startKeyframe + static_cast<int>(N*0.75))->bone_rotation[bone];
            vector r4 = endPosture->bone_rotation[bone];

            controlRot[bone][0][0] = r1.x(); controlRot[bone][0][1] = r1.y(); controlRot[bone][0][2] = r1.z();
            controlRot[bone][1][0] = r2.x(); controlRot[bone][1][1] = r2.y(); controlRot[bone][1][2] = r2.z();
            controlRot[bone][2][0] = r3.x(); controlRot[bone][2][1] = r3.y(); controlRot[bone][2][2] = r3.z();
            controlRot[bone][3][0] = r4.x(); controlRot[bone][3][1] = r4.y(); controlRot[bone][3][2] = r4.z();

            for(int i=0; i<4; i++)
                for(int j=0; j<3; j++)
                {
                    basisControlRot[bone][i][j] = 0;
                    for(int k=0; k<4; k++)
                        basisControlRot[bone][i][j] += BEZIER_BASIS[i*4 + k] * controlRot[bone][k][j];
                }
        }

        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

            // interpolate root position
            double u[4] {t*t*t, t*t, t, 1.0};

            // interpolate root position using Bezier
            vector p(0, 0, 0);
            for (int i = 0; i < 4; i++){
                for(int j = 0; j<3; j++){
                    p[j] += u[i]*basisControlPos[i][j];
                }
            }
            interpolatedPosture.root_pos = p;

            // interpolate bone rotations using Bezier
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
                vector boneRot(0, 0, 0);
                for (int i = 0; i < 4; i++){
                    for(int j = 0; j<3; j++){
                        boneRot[j] += u[i]*basisControlRot[bone][i][j];
                    }
                }
                interpolatedPosture.bone_rotation[bone] = boneRot;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
                Quaternion<double> a,b,q;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, a);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, b);
                q = Slerp(t, a, b);
                Quaternion2Euler(q, interpolatedPosture.bone_rotation[bone].p);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double R[9];
    Euler2Rotation(angles, R);
    q = Quaternion<double>::Matrix2Quaternion(R);
    q.MoveToRightHalfSphere();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    double R[9];
    q.Quaternion2Matrix(R);
    Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // Spherical Linear Interpolation between two quaternions
  Quaternion<double> result;

  // Get quaternion components
  double x0 = qStart.Getx();
  double y0 = qStart.Gety();
  double z0 = qStart.Getz();
  double w0 = qStart.Gets();

  double x1 = qEnd_.Getx();
  double y1 = qEnd_.Gety();
  double z1 = qEnd_.Getz();
  double w1 = qEnd_.Gets();

  // Calculate dot product
  double dot = x0*x1 + y0*y1 + z0*z1 + w0*w1;

  // If dot product is negative, negate one quaternion to take shorter path
  if (dot < 0.0) {
    x1 = -x1;
    y1 = -y1;
    z1 = -z1;
    w1 = -w1;
    dot = -dot;
  }

  // Clamp dot product to [-1, 1] to avoid NaN from acos
  if (dot > 1.0) dot = 1.0;
  if (dot < -1.0) dot = -1.0;

  // Calculate the angle between the quaternions
  double theta = acos(dot);
  double sinTheta = sin(theta);

  // If the angle is very small, use linear interpolation to avoid division by zero
  if (sinTheta < 1e-6) {
    result.Set(
      w0 * (1.0 - t) + w1 * t,
      x0 * (1.0 - t) + x1 * t,
      y0 * (1.0 - t) + y1 * t,
      z0 * (1.0 - t) + z1 * t
    );
  } else {
    // Calculate interpolation coefficients
    double coeff1 = sin((1.0 - t) * theta) / sinTheta;
    double coeff2 = sin(t * theta) / sinTheta;

    // Perform spherical linear interpolation
    result.Set(
      coeff1 * w0 + coeff2 * w1,
      coeff1 * x0 + coeff2 * x1,
      coeff1 * y0 + coeff2 * y1,
      coeff1 * z0 + coeff2 * z1
    );
  }

  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

