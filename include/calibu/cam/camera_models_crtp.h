/*
  This file is part of the Calibu Project.
  https://github.com/arpg/Calibu

  Copyright (C) 2015
  Steven Lovegrove,
  Nima Keivan
  Christoffer Heckman,
  Gabe Sibley,
  University of Colorado at Boulder,
  George Washington University.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

/**
 * This file defines various camera models via including their files.
 *
 *
 * NOTES ON RADIAL DISTORTION MODELS (Poly2, Poly3, Rational3, etc.)
 * Generalized radial distortion model:
 *   x_d = f(r_u)*x_u                           (1)
 *   y_d = f(r_u)*y_u
 *   r_d = sqrt(x_d^2 + y_d^2)
 *   r_d = sqrt((f(r_u)*x_u)^2 + (f(r_u)*y_u)^2)
 *   r_d = sqrt((f(r_u))^2*(x_u^2*y_u^2))       (2)
 * Recall that r_u = x_u^2*y_u^2                (3)
 * So, if we want to consider purely the radii, substitute (3) into (2):
 *   r_d = r_u*f(r_u)                           (4)
 * here, r_d is known, so if we find minimum of (r_u*f(r_u)-r_d)^2 we are effectively finding
 * the zeroes of equation (4), and several models use the Newton method to approximate these quickly.
 * Using the found r_u, we use r_u / r_d = 1 / f(r_u) to compute the factor,
 * so we can compute x_u = (r_u / r_d) * x_d.
 */

#pragma once
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_utils.h>
#include <iostream>

/// Add camera model templates.
#include <calibu/cam/camera_models_poly.h>
#include <calibu/cam/camera_models_kb4.h>
#include <calibu/cam/camera_models_rational.h>
