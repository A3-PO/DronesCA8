#ifndef __c1_controller_v1_h__
#define __c1_controller_v1_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_controller_v1InstanceStruct
#define typedef_SFc1_controller_v1InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_controller_v1;
  real_T *c1_x_drone;
  real_T *c1_alpha_d;
  real_T *c1_y_drone;
  real_T *c1_z_drone;
  real_T *c1_theta_d;
  real_T *c1_phi_d;
  real_T *c1_x_gs;
  real_T *c1_y_gs;
  real_T *c1_z_gs;
  real_T *c1_theta_gs;
  real_T *c1_phi_gs;
  real_T *c1_alpha_gs;
  real_T *c1_gamma_d;
  real_T *c1_gamma_gs;
  real_T *c1_opt_theta_gs;
  real_T *c1_opt_theta_d;
  real_T *c1_opt_phi_gs;
  real_T *c1_opt_phi_d;
} SFc1_controller_v1InstanceStruct;

#endif                                 /*typedef_SFc1_controller_v1InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_controller_v1_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_controller_v1_get_check_sum(mxArray *plhs[]);
extern void c1_controller_v1_method_dispatcher(SimStruct *S, int_T method, void *
  data);

#endif
