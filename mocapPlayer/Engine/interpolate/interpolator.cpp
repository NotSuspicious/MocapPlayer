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

double Interpolator::GetAverageJerk(Motion* motionA, Motion* motionB, int startIndex) {
    // Jerk is the rate of change of acceleration
    // jerk = (a2 - a1) / dt, where a = (v2 - v1) / dt
    // For frame-based animation with dt = 1 frame:
    // jerk = (pos[i+2] - 2*pos[i+1] + pos[i]) - (pos[i+1] - 2*pos[i] + pos[i-1])

    // Assume 120 fps for real-time conversion
    const double FPS = 120.0;
    const double dt = 1.0 / FPS;  // time step in seconds
    // For jerk: jerk_physical = jerk_frame / dt^3
    // Since we compute jerk in frame units (dt=1), we need to divide by dt^3 to get physical units
    const double dt3 = dt * dt * dt;  // dt^3 for jerk scaling (very small: ~5.787e-7)

    double totalJerk = 0.0;
    int jerkCount = 0;

    // Need at least 4 frames to compute jerk (frame i-1, i, i+1, i+2)
    int maxFrames = std::min(motionA->GetNumFrames(), motionB->GetNumFrames());

    for (int frame = startIndex + 1; frame < maxFrames - 2; frame++) {
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
            // Get 4 consecutive frames from both motions
            vector a0 = motionA->GetPosture(frame - 1)->bone_rotation[bone];
            vector a1 = motionA->GetPosture(frame)->bone_rotation[bone];
            vector a2 = motionA->GetPosture(frame + 1)->bone_rotation[bone];
            vector a3 = motionA->GetPosture(frame + 2)->bone_rotation[bone];

            vector b0 = motionB->GetPosture(frame - 1)->bone_rotation[bone];
            vector b1 = motionB->GetPosture(frame)->bone_rotation[bone];
            vector b2 = motionB->GetPosture(frame + 1)->bone_rotation[bone];
            vector b3 = motionB->GetPosture(frame + 2)->bone_rotation[bone];

            // Compute acceleration at frame i: a[i] = (pos[i+1] - 2*pos[i] + pos[i-1])
            vector aAcc_i = (a2 - a1 * 2.0 + a0);
            vector aAcc_i1 = (a3 - a2 * 2.0 + a1);

            vector bAcc_i = (b2 - b1 * 2.0 + b0);
            vector bAcc_i1 = (b3 - b2 * 2.0 + b1);

            // Jerk = change in acceleration
            vector aJerk = aAcc_i1 - aAcc_i;
            vector bJerk = bAcc_i1 - bAcc_i;

            // Difference in jerk between motions
            vector jerkDiff = bJerk - aJerk;

            // Accumulate magnitude of jerk difference
            // Divide by dt^3 to convert from frame units to physical units (degrees/second^3)
            totalJerk += jerkDiff.length() / dt3;
            jerkCount++;
        }
    }

    if (jerkCount == 0) {
        return 0.0;
    }

    return totalJerk / jerkCount;
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  // Sample input motion for comparison
  FILE* INPUT_LFEMUR = fopen("INPUT_LFEMUR.txt", "w");
  FILE* INPUT_ROOT = fopen("INPUT_ROOT.txt", "w");

  if (!INPUT_LFEMUR || !INPUT_ROOT) {
    printf("Warning: Could not open input sampling files\n");
  }

  // Sample frames 600-800 for LFEMUR
  if (INPUT_LFEMUR) {
    for (int frame = 600; frame <= 800; frame++) {
      if (frame < pInputMotion->GetNumFrames()) {
        Posture* posture = pInputMotion->GetPosture(frame);
        double yAxis = posture->bone_rotation[Posture::LFEMUR_INDEX].x();
        double xAxis = frame;
        fprintf(INPUT_LFEMUR, "%f,%f\n", xAxis, yAxis);
      }
    }
    fflush(INPUT_LFEMUR);
    fclose(INPUT_LFEMUR);
    printf("Input LFEMUR data written to INPUT_LFEMUR.txt (frames 600-800)\n");
  }

  // Sample frames 200-500 for ROOT
  if (INPUT_ROOT) {
    for (int frame = 200; frame <= 500; frame++) {
      if (frame < pInputMotion->GetNumFrames()) {
        Posture* posture = pInputMotion->GetPosture(frame);
        double yAxis = posture->bone_rotation[Posture::ROOT_INDEX].z();
        double xAxis = frame;
        fprintf(INPUT_ROOT, "%f,%f\n", xAxis, yAxis);
      }
    }
    fflush(INPUT_ROOT);
    fclose(INPUT_ROOT);
    printf("Input ROOT data written to INPUT_ROOT.txt (frames 200-500)\n");
  }

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
  auto startTime = std::chrono::high_resolution_clock::now();

  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  // Open files for logging
  FILE* LELFEMUR = fopen("LELFEMUR.txt", "w");
  FILE* LEROOT = fopen("LEROOT.txt", "w");

  if (!LELFEMUR || !LEROOT) {
    printf("Warning: Could not open LinearInterpolationEuler output files\n");
  }

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
      int currentKeyframe = startKeyframe + frame;
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

        // Log LFEMUR frames 600-800
        if (currentKeyframe >= 600 && currentKeyframe <= 800){
          if (bone == Posture::LFEMUR_INDEX){
            double yAxis = interpolatedPosture.bone_rotation[bone].x();
            double xAxis = currentKeyframe;
            fprintf(LELFEMUR, "%f,%f\n", xAxis, yAxis);
            fflush(LELFEMUR);
          }
        }

        // Log ROOT frames 200-500
        if (currentKeyframe >= 200 && currentKeyframe <= 500){
          if (bone == Posture::ROOT_INDEX){
            double yAxis = interpolatedPosture.bone_rotation[bone].z();
            double xAxis = currentKeyframe;
            fprintf(LEROOT, "%f,%f\n", xAxis, yAxis);
            fflush(LEROOT);
          }
        }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    // Log the endKeyframe (keyframe data)
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
    {
      // Log LFEMUR frames 600-800
      if (endKeyframe >= 600 && endKeyframe <= 800){
        if (bone == Posture::LFEMUR_INDEX){
          double yAxis = endPosture->bone_rotation[bone].x();
          double xAxis = endKeyframe;
          fprintf(LELFEMUR, "%f,%f\n", xAxis, yAxis);
          fflush(LELFEMUR);
        }
      }

      // Log ROOT frames 200-500
      if (endKeyframe >= 200 && endKeyframe <= 500){
        if (bone == Posture::ROOT_INDEX){
          double yAxis = endPosture->bone_rotation[bone].z();
          double xAxis = endKeyframe;
          fprintf(LEROOT, "%f,%f\n", xAxis, yAxis);
          fflush(LEROOT);
        }
      }
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  // Close files with proper flushing
  if (LELFEMUR) {
    fflush(LELFEMUR);
    fclose(LELFEMUR);
    printf("LFEMUR data written to LELFEMUR.txt\n");
  }
  if (LEROOT) {
    fflush(LEROOT);
    fclose(LEROOT);
    printf("ROOT data written to LEROOT.txt\n");
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = endTime - startTime;
  m_linearEulerTime = elapsed.count();
  printf("LinearInterpolationEuler completed in %.6f seconds\n", m_linearEulerTime);
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
    auto startTime = std::chrono::high_resolution_clock::now();

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    // Open files for logging
    FILE* BEELFEMUR = fopen("BEELFEMUR.txt", "w");
    FILE* BEEROOT = fopen("BEEROOT.txt", "w");

    if (!BEELFEMUR || !BEEROOT) {
      printf("Warning: Could not open BezierInterpolationEuler output files\n");
    }

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
            int currentKeyframe = startKeyframe + frame;
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

                // Log LFEMUR frames 600-800
                if (currentKeyframe >= 600 && currentKeyframe <= 800){
                  if (bone == Posture::LFEMUR_INDEX){
                    double yAxis = interpolatedPosture.bone_rotation[bone].x();
                    double xAxis = currentKeyframe;
                    fprintf(BEELFEMUR, "%f,%f\n", xAxis, yAxis);
                    fflush(BEELFEMUR);
                  }
                }

                // Log ROOT frames 200-500
                if (currentKeyframe >= 200 && currentKeyframe <= 500){
                  if (bone == Posture::ROOT_INDEX){
                    double yAxis = interpolatedPosture.bone_rotation[bone].z();
                    double xAxis = currentKeyframe;
                    fprintf(BEEROOT, "%f,%f\n", xAxis, yAxis);
                    fflush(BEEROOT);
                  }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        // Log the endKeyframe (keyframe data)
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
          // Log LFEMUR frames 600-800
          if (endKeyframe >= 600 && endKeyframe <= 800){
            if (bone == Posture::LFEMUR_INDEX){
              double yAxis = endPosture->bone_rotation[bone].x();
              double xAxis = endKeyframe;
              fprintf(BEELFEMUR, "%f,%f\n", xAxis, yAxis);
              fflush(BEELFEMUR);
            }
          }

          // Log ROOT frames 200-500
          if (endKeyframe >= 200 && endKeyframe <= 500){
            if (bone == Posture::ROOT_INDEX){
              double yAxis = endPosture->bone_rotation[bone].z();
              double xAxis = endKeyframe;
              fprintf(BEEROOT, "%f,%f\n", xAxis, yAxis);
              fflush(BEEROOT);
            }
          }
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    // Close files with proper flushing
    if (BEELFEMUR) {
      fflush(BEELFEMUR);
      fclose(BEELFEMUR);
      printf("LFEMUR data written to BEELFEMUR.txt\n");
    }
    if (BEEROOT) {
      fflush(BEEROOT);
      fclose(BEEROOT);
      printf("ROOT data written to BEEROOT.txt\n");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    m_bezierEulerTime = elapsed.count();
    printf("BezierInterpolationEuler completed in %.6f seconds\n", m_bezierEulerTime);
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    // Open files for logging
    FILE* LIQLFEMUR = fopen("LIQLFEMUR.txt", "w");
    FILE* LIQROOT = fopen("LIQROOT.txt", "w");

    if (!LIQLFEMUR || !LIQROOT) {
      printf("Warning: Could not open LinearInterpolationQuaternion output files\n");
    }

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
            int currentKeyframe = startKeyframe + frame;
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

                // Log LFEMUR frames 600-800
                if (currentKeyframe >= 600 && currentKeyframe <= 800){
                  if (bone == Posture::LFEMUR_INDEX){
                    double yAxis = interpolatedPosture.bone_rotation[bone].x();
                    double xAxis = currentKeyframe;
                    fprintf(LIQLFEMUR, "%f,%f\n", xAxis, yAxis);
                    fflush(LIQLFEMUR);
                  }
                }

                // Log ROOT frames 200-500
                if (currentKeyframe >= 200 && currentKeyframe <= 500){
                  if (bone == Posture::ROOT_INDEX){
                    double yAxis = interpolatedPosture.bone_rotation[bone].z();
                    double xAxis = currentKeyframe;
                    fprintf(LIQROOT, "%f,%f\n", xAxis, yAxis);
                    fflush(LIQROOT);
                  }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        // Log the endKeyframe (keyframe data)
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
          // Log LFEMUR frames 600-800
          if (endKeyframe >= 600 && endKeyframe <= 800){
            if (bone == Posture::LFEMUR_INDEX){
              double yAxis = endPosture->bone_rotation[bone].x();
              double xAxis = endKeyframe;
              fprintf(LIQLFEMUR, "%f,%f\n", xAxis, yAxis);
              fflush(LIQLFEMUR);
            }
          }

          // Log ROOT frames 200-500
          if (endKeyframe >= 200 && endKeyframe <= 500){
            if (bone == Posture::ROOT_INDEX){
              double yAxis = endPosture->bone_rotation[bone].z();
              double xAxis = endKeyframe;
              fprintf(LIQROOT, "%f,%f\n", xAxis, yAxis);
              fflush(LIQROOT);
            }
          }
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    // Close files with proper flushing
    if (LIQLFEMUR) {
      fflush(LIQLFEMUR);
      fclose(LIQLFEMUR);
      printf("LFEMUR data written to LIQLFEMUR.txt\n");
    }
    if (LIQROOT) {
      fflush(LIQROOT);
      fclose(LIQROOT);
      printf("ROOT data written to LIQROOT.txt\n");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    m_linearQuaternionTime = elapsed.count();
    printf("LinearInterpolationQuaternion completed in %.6f seconds\n", m_linearQuaternionTime);
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    // Open file for logging bone rotation data
    FILE* BQLFEMUR = fopen("BQLFEMUR.txt", "w");
    FILE* BQROOT = fopen("BQROOT.txt", "w");

    if (!BQLFEMUR || !BQROOT) {
        printf("Warning: Could not open bone_rotations.txt for writing\n");
    }

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // Bezier control points for quaternion interpolation
        std::vector<std::vector<Quaternion<double>>> controlPoints(MAX_BONES_IN_ASF_FILE);

        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            Quaternion<double> qCurrent, qNext;
            Euler2Quaternion(startPosture->bone_rotation[bone].p, qCurrent);
            Euler2Quaternion(endPosture->bone_rotation[bone].p, qNext);

            Quaternion<double> p1, p2;

            // Check if we have both previous and next keyframes for proper tangent computation
            bool hasPrev = (startKeyframe >= N + 1);
            bool hasNext = (endKeyframe + N + 1 < inputLength);

            if (hasPrev && hasNext)
            {
                // We have qn-1, qn, qn+1, qn+2 - compute with full tangent info
                Quaternion<double> qPrevKeyframe;
                Quaternion<double> qNextKeyframe;

                Euler2Quaternion(pInputMotion->GetPosture(startKeyframe - (N+1))->bone_rotation[bone].p, qPrevKeyframe);
                Euler2Quaternion(pInputMotion->GetPosture(endKeyframe + (N+1))->bone_rotation[bone].p, qNextKeyframe);

                // Lecture formula for an (tangent at qn):
                // an = Slerp(Slerp(qn-1, qn, 2.0), qn+1, 0.5)
                Quaternion<double> temp_n = Slerp(2.0, qPrevKeyframe, qCurrent);
                Quaternion<double> an = Slerp(0.5, temp_n, qNext);

                // Lecture formula for bn (tangent at qn+1):
                // bn = Slerp(Slerp(qn, qn+1, 2.0), qn+2, 0.5)
                Quaternion<double> temp_n1 = Slerp(2.0, qCurrent, qNext);
                Quaternion<double> bn = Slerp(0.5, temp_n1, qNextKeyframe);

                // Control points:
                // p1 = Slerp(qn, an, 1.0/3.0)
                // p2 = Slerp(qn+1, bn, -1.0/3.0)
                p1 = Slerp(1.0/3.0, qCurrent, an);
                p2 = Slerp(-1.0/3.0, qNext, bn);
            }
            else if (hasPrev)
            {
                // Have previous but not next: use qPrev and qCurrent to estimate tangent
                Quaternion<double> qPrevKeyframe;
                Euler2Quaternion(pInputMotion->GetPosture(startKeyframe - (N+1))->bone_rotation[bone].p, qPrevKeyframe);

                // Estimate tangent at qCurrent from previous segment
                Quaternion<double> temp = Slerp(2.0, qPrevKeyframe, qCurrent);
                Quaternion<double> an = Slerp(0.5, temp, qNext);

                // Use linear fallback for qNext tangent
                p1 = Slerp(1.0/3.0, qCurrent, an);
                p2 = Slerp(2.0/3.0, qCurrent, qNext);
            }
            else if (hasNext)
            {
                // Have next but not previous: use qCurrent and qNext to estimate tangent
                Quaternion<double> qNextKeyframe;
                Euler2Quaternion(pInputMotion->GetPosture(endKeyframe + (N+1))->bone_rotation[bone].p, qNextKeyframe);

                // Use linear fallback for qCurrent tangent
                Quaternion<double> bn = Slerp(0.5, Slerp(2.0, qCurrent, qNext), qNextKeyframe);

                p1 = Slerp(1.0/3.0, qCurrent, qNext);
                p2 = Slerp(-1.0/3.0, qNext, bn);
            }
            else
            {
                // At start/end of motion, use simple linear interpolation for control points
                p1 = Slerp(1.0/3.0, qCurrent, qNext);
                p2 = Slerp(2.0/3.0, qCurrent, qNext);
            }

            controlPoints[bone].push_back(p1);
            controlPoints[bone].push_back(p2);
        }

        // Interpolate frames between keyframes
        for(int frame=1; frame<=N; frame++)
        {
            int currentKeyframe = startKeyframe+frame;

            double t = 1.0 * frame / (N+1);
            Posture interpolatedPosture;

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // Use Bezier interpolation with computed control points
                Quaternion<double> p0, p1, p2, p3;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, p0);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, p3);
                p1 = controlPoints[bone][0];
                p2 = controlPoints[bone][1];

                // Apply De Casteljau algorithm
                Quaternion<double> interpolatedQ = DeCasteljauQuaternion(t, p0, p1, p2, p3);
                Quaternion2Euler(interpolatedQ, interpolatedPosture.bone_rotation[bone].p);

                //Plot lfemur - frames 600-800
                if (currentKeyframe >= 600 && currentKeyframe <= 800){
                    if (bone == Posture::LFEMUR_INDEX){
                        double yAxis = interpolatedPosture.bone_rotation[bone].x();
                        double xAxis = currentKeyframe;
                        fprintf(BQLFEMUR, "%f,%f\n", xAxis, yAxis);
                        fflush(BQLFEMUR);
                    }
                }

                //Plot root - frames 200-500
                if (currentKeyframe >= 200 && currentKeyframe <= 500){
                    if (bone == Posture::ROOT_INDEX){
                        double yAxis = interpolatedPosture.bone_rotation[bone].z();
                        double xAxis = currentKeyframe;
                        fprintf(BQROOT, "%f,%f\n", xAxis, yAxis);
                        fflush(BQROOT);
                    }
                }
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        // Log the endKeyframe (keyframe data)
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
          // Log LFEMUR frames 600-800
          if (endKeyframe >= 600 && endKeyframe <= 800){
            if (bone == Posture::LFEMUR_INDEX){
              double yAxis = endPosture->bone_rotation[bone].x();
              double xAxis = endKeyframe;
              fprintf(BQLFEMUR, "%f,%f\n", xAxis, yAxis);
              fflush(BQLFEMUR);
            }
          }

          // Log ROOT frames 200-500
          if (endKeyframe >= 200 && endKeyframe <= 500){
            if (bone == Posture::ROOT_INDEX){
              double yAxis = endPosture->bone_rotation[bone].z();
              double xAxis = endKeyframe;
              fprintf(BQROOT, "%f,%f\n", xAxis, yAxis);
              fflush(BQROOT);
            }
          }
        }

        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    // Close files with proper flushing
    if (BQLFEMUR) {
        fflush(BQLFEMUR);
        fclose(BQLFEMUR);
        printf("LFEMUR data written to BQLFEMUR.txt\n");
    }
    if (BQROOT) {
        fflush(BQROOT);
        fclose(BQROOT);
        printf("ROOT data written to BQROOT.txt\n");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    m_bezierQuaternionTime = elapsed.count();
    printf("BezierInterpolationQuaternion completed in %.6f seconds\n", m_bezierQuaternionTime);
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

Quaternion<double> Interpolator::Slerp(double t, const Quaternion<double> & qStart, const Quaternion<double> & qEnd_)
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
    // Calculate interpolation coefficients (works for any t, including t > 1 and t < 0)
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
  Quaternion<double> Q0,Q1,Q2,R0,R1;

  Q0 = Slerp(t, p0,p1);
  Q1 = Slerp(t, p1,p2);
  Q2 = Slerp(t, p2,p3);
  R0 = Slerp(t, Q0,Q1);
  R1 = Slerp(t, Q1,Q2);
  result = Slerp(t, R0, R1);
  return result;
}

