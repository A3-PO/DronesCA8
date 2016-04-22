/* Include files */

#include <stddef.h>
#include "blas.h"
#include "controller_v1_sfun.h"
#include "c1_controller_v1.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "controller_v1_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[20] = { "nargin", "nargout", "x_drone",
  "y_drone", "z_drone", "theta_d", "phi_d", "x_gs", "y_gs", "z_gs", "theta_gs",
  "phi_gs", "alpha_d", "alpha_gs", "gamma_d", "gamma_gs", "opt_theta_gs",
  "opt_theta_d", "opt_phi_gs", "opt_phi_d" };

/* Function Declarations */
static void initialize_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void initialize_params_c1_controller_v1(SFc1_controller_v1InstanceStruct *
  chartInstance);
static void enable_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void disable_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_controller_v1
  (SFc1_controller_v1InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_controller_v1
  (SFc1_controller_v1InstanceStruct *chartInstance);
static void set_sim_state_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void sf_gateway_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void mdl_start_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void c1_chartstep_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void initSimStructsc1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_b_opt_phi_d, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static real_T c1_abs(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                     c1_x);
static real_T c1_atan2(SFc1_controller_v1InstanceStruct *chartInstance, real_T
  c1_y, real_T c1_x);
static void c1_scalarEg(SFc1_controller_v1InstanceStruct *chartInstance);
static void c1_dimagree(SFc1_controller_v1InstanceStruct *chartInstance);
static real_T c1_mpower(SFc1_controller_v1InstanceStruct *chartInstance, real_T
  c1_a);
static void c1_error(SFc1_controller_v1InstanceStruct *chartInstance);
static real_T c1_sqrt(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                      c1_x);
static void c1_b_error(SFc1_controller_v1InstanceStruct *chartInstance);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_c_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_d_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_controller_v1, const char_T
  *c1_identifier);
static uint8_T c1_e_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sqrt(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                      *c1_x);
static void init_dsm_address_info(SFc1_controller_v1InstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc1_controller_v1InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_controller_v1(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_controller_v1 = 0U;
}

static void initialize_params_c1_controller_v1(SFc1_controller_v1InstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_controller_v1
  (SFc1_controller_v1InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_controller_v1
  (SFc1_controller_v1InstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  uint8_T c1_i_hoistedGlobal;
  uint8_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(9, 1), false);
  c1_hoistedGlobal = *chartInstance->c1_alpha_d;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *chartInstance->c1_alpha_gs;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *chartInstance->c1_gamma_d;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *chartInstance->c1_gamma_gs;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = *chartInstance->c1_opt_phi_d;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = *chartInstance->c1_opt_phi_gs;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = *chartInstance->c1_opt_theta_d;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = *chartInstance->c1_opt_theta_gs;
  c1_h_u = c1_h_hoistedGlobal;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = chartInstance->c1_is_active_c1_controller_v1;
  c1_i_u = c1_i_hoistedGlobal;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 8, c1_j_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  *chartInstance->c1_alpha_d = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("alpha_d", c1_u, 0)), "alpha_d");
  *chartInstance->c1_alpha_gs = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("alpha_gs", c1_u, 1)), "alpha_gs");
  *chartInstance->c1_gamma_d = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("gamma_d", c1_u, 2)), "gamma_d");
  *chartInstance->c1_gamma_gs = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("gamma_gs", c1_u, 3)), "gamma_gs");
  *chartInstance->c1_opt_phi_d = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("opt_phi_d", c1_u, 4)), "opt_phi_d");
  *chartInstance->c1_opt_phi_gs = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("opt_phi_gs", c1_u, 5)), "opt_phi_gs");
  *chartInstance->c1_opt_theta_d = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("opt_theta_d", c1_u, 6)), "opt_theta_d");
  *chartInstance->c1_opt_theta_gs = c1_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("opt_theta_gs", c1_u, 7)), "opt_theta_gs");
  chartInstance->c1_is_active_c1_controller_v1 = c1_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c1_controller_v1", c1_u,
       8)), "is_active_c1_controller_v1");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_controller_v1(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_phi_gs, 9U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_theta_gs, 8U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_z_gs, 7U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_y_gs, 6U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_x_gs, 5U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_phi_d, 4U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_theta_d, 3U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_z_drone, 2U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_y_drone, 1U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_x_drone, 0U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_controller_v1(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_controller_v1MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_alpha_d, 10U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_alpha_gs, 11U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_gamma_d, 12U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_gamma_gs, 13U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_opt_theta_gs, 14U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_opt_theta_d, 15U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_opt_phi_gs, 16U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_opt_phi_d, 17U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
}

static void mdl_start_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  real_T c1_f_hoistedGlobal;
  real_T c1_g_hoistedGlobal;
  real_T c1_h_hoistedGlobal;
  real_T c1_i_hoistedGlobal;
  real_T c1_j_hoistedGlobal;
  real_T c1_b_x_drone;
  real_T c1_b_y_drone;
  real_T c1_b_z_drone;
  real_T c1_b_theta_d;
  real_T c1_b_phi_d;
  real_T c1_b_x_gs;
  real_T c1_b_y_gs;
  real_T c1_b_z_gs;
  real_T c1_b_theta_gs;
  real_T c1_b_phi_gs;
  uint32_T c1_debug_family_var_map[20];
  real_T c1_nargin = 10.0;
  real_T c1_nargout = 8.0;
  real_T c1_b_alpha_d;
  real_T c1_b_alpha_gs;
  real_T c1_b_gamma_d;
  real_T c1_b_gamma_gs;
  real_T c1_b_opt_theta_gs;
  real_T c1_b_opt_theta_d;
  real_T c1_b_opt_phi_gs;
  real_T c1_b_opt_phi_d;
  real_T c1_d0;
  real_T c1_d1;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_x_drone;
  c1_b_hoistedGlobal = *chartInstance->c1_y_drone;
  c1_c_hoistedGlobal = *chartInstance->c1_z_drone;
  c1_d_hoistedGlobal = *chartInstance->c1_theta_d;
  c1_e_hoistedGlobal = *chartInstance->c1_phi_d;
  c1_f_hoistedGlobal = *chartInstance->c1_x_gs;
  c1_g_hoistedGlobal = *chartInstance->c1_y_gs;
  c1_h_hoistedGlobal = *chartInstance->c1_z_gs;
  c1_i_hoistedGlobal = *chartInstance->c1_theta_gs;
  c1_j_hoistedGlobal = *chartInstance->c1_phi_gs;
  c1_b_x_drone = c1_hoistedGlobal;
  c1_b_y_drone = c1_b_hoistedGlobal;
  c1_b_z_drone = c1_c_hoistedGlobal;
  c1_b_theta_d = c1_d_hoistedGlobal;
  c1_b_phi_d = c1_e_hoistedGlobal;
  c1_b_x_gs = c1_f_hoistedGlobal;
  c1_b_y_gs = c1_g_hoistedGlobal;
  c1_b_z_gs = c1_h_hoistedGlobal;
  c1_b_theta_gs = c1_i_hoistedGlobal;
  c1_b_phi_gs = c1_j_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 20U, 20U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_x_drone, 2U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_y_drone, 3U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_z_drone, 4U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_theta_d, 5U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_phi_d, 6U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_x_gs, 7U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_y_gs, 8U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_z_gs, 9U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_theta_gs, 10U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_phi_gs, 11U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_alpha_d, 12U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_alpha_gs, 13U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_gamma_d, 14U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_gamma_gs, 15U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_opt_theta_gs, 16U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_opt_theta_d, 17U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_opt_phi_gs, 18U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_opt_phi_d, 19U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 48);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c1_b_theta_gs, 0.0, -1,
        2U, c1_b_theta_gs < 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
    c1_b_theta_gs += 6.2831853071795862;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c1_b_theta_d, 0.0, -1, 2U,
        c1_b_theta_d < 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
    c1_b_theta_d += 6.2831853071795862;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
  if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c1_b_phi_gs, 0.0, -1, 4U,
        c1_b_phi_gs > 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 57);
    c1_b_phi_gs = 1.5707963267948966 - c1_b_phi_gs;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 59);
    c1_b_phi_gs = 1.5707963267948966 + c1_abs(chartInstance, c1_b_phi_gs);
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 61);
  if (CV_EML_IF(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 3, c1_b_phi_d, 0.0, -1, 4U,
        c1_b_phi_d > 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 62);
    c1_b_phi_d = 1.5707963267948966 - c1_b_phi_d;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
    c1_b_phi_d = 1.5707963267948966 + c1_abs(chartInstance, c1_b_phi_d);
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 69);
  if (CV_EML_IF(0, 1, 4, CV_RELATIONAL_EVAL(4U, 0U, 4, c1_b_y_drone, c1_b_y_gs,
        -1, 4U, c1_b_y_drone > c1_b_y_gs))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 70);
    c1_b_opt_theta_gs = c1_atan2(chartInstance, c1_b_y_drone - c1_b_y_gs,
      c1_b_x_drone - c1_b_x_gs);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 71);
    c1_b_opt_theta_d = 3.1415926535897931 + c1_b_opt_theta_gs;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 73);
    c1_b_opt_theta_d = c1_atan2(chartInstance, c1_b_y_gs - c1_b_y_drone,
      c1_b_x_gs - c1_b_x_drone);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 74);
    c1_b_opt_theta_gs = 3.1415926535897931 + c1_b_opt_theta_d;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 78);
  c1_b_alpha_gs = c1_b_opt_theta_gs - c1_b_theta_gs;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 80);
  c1_b_alpha_d = c1_b_opt_theta_d - c1_b_theta_d;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 84);
  if (CV_EML_IF(0, 1, 5, CV_RELATIONAL_EVAL(4U, 0U, 5, c1_b_alpha_gs, 180.0, -1,
        4U, c1_b_alpha_gs > 180.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 85);
    c1_b_alpha_gs = 360.0 - c1_b_alpha_gs;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 87);
  if (CV_EML_IF(0, 1, 6, CV_RELATIONAL_EVAL(4U, 0U, 6, c1_b_alpha_d, 180.0, -1,
        4U, c1_b_alpha_d > 180.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 88);
    c1_b_alpha_d = 360.0 - c1_b_alpha_d;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 94);
  c1_d0 = c1_mpower(chartInstance, c1_abs(chartInstance, c1_b_x_gs -
    c1_b_x_drone)) + c1_mpower(chartInstance, c1_abs(chartInstance, c1_b_y_gs -
    c1_b_y_drone));
  c1_b_sqrt(chartInstance, &c1_d0);
  c1_b_opt_phi_gs = 1.5707963267948966 - c1_atan2(chartInstance, c1_b_z_drone -
    c1_b_z_gs, c1_d0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 96);
  c1_d1 = c1_mpower(chartInstance, c1_abs(chartInstance, c1_b_x_gs -
    c1_b_x_drone)) + c1_mpower(chartInstance, c1_abs(chartInstance, c1_b_y_gs -
    c1_b_y_drone));
  c1_b_sqrt(chartInstance, &c1_d1);
  c1_b_opt_phi_d = 1.5707963267948966 + c1_atan2(chartInstance, c1_b_z_drone -
    c1_b_z_gs, c1_d1);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 99);
  c1_b_gamma_gs = c1_b_opt_phi_gs - c1_b_phi_gs;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 101);
  c1_b_gamma_d = c1_b_opt_phi_d - c1_b_phi_d;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -101);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c1_alpha_d = c1_b_alpha_d;
  *chartInstance->c1_alpha_gs = c1_b_alpha_gs;
  *chartInstance->c1_gamma_d = c1_b_gamma_d;
  *chartInstance->c1_gamma_gs = c1_b_gamma_gs;
  *chartInstance->c1_opt_theta_gs = c1_b_opt_theta_gs;
  *chartInstance->c1_opt_theta_d = c1_b_opt_theta_d;
  *chartInstance->c1_opt_phi_gs = c1_b_opt_phi_gs;
  *chartInstance->c1_opt_phi_d = c1_b_opt_phi_d;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_controller_v1(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controller_v1InstanceStruct *chartInstance;
  chartInstance = (SFc1_controller_v1InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_b_opt_phi_d, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_opt_phi_d),
    &c1_thisId);
  sf_mex_destroy(&c1_b_opt_phi_d);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d2, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d2;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_opt_phi_d;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_controller_v1InstanceStruct *chartInstance;
  chartInstance = (SFc1_controller_v1InstanceStruct *)chartInstanceVoid;
  c1_b_opt_phi_d = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_opt_phi_d),
    &c1_thisId);
  sf_mex_destroy(&c1_b_opt_phi_d);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_controller_v1_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_abs(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                     c1_x)
{
  real_T c1_b_x;
  real_T c1_c_x;
  (void)chartInstance;
  c1_b_x = c1_x;
  c1_c_x = c1_b_x;
  return muDoubleScalarAbs(c1_c_x);
}

static real_T c1_atan2(SFc1_controller_v1InstanceStruct *chartInstance, real_T
  c1_y, real_T c1_x)
{
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_c_y;
  real_T c1_c_x;
  (void)chartInstance;
  c1_b_x = c1_y;
  c1_b_y = c1_x;
  c1_c_y = c1_b_x;
  c1_c_x = c1_b_y;
  return muDoubleScalarAtan2(c1_c_y, c1_c_x);
}

static void c1_scalarEg(SFc1_controller_v1InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_dimagree(SFc1_controller_v1InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c1_mpower(SFc1_controller_v1InstanceStruct *chartInstance, real_T
  c1_a)
{
  real_T c1_c;
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_x;
  real_T c1_d_a;
  boolean_T c1_p;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_x = c1_c_a;
  c1_d_a = c1_x;
  c1_c = c1_d_a * c1_d_a;
  c1_p = false;
  if (c1_p) {
    c1_error(chartInstance);
  }

  return c1_c;
}

static void c1_error(SFc1_controller_v1InstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 31), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_y));
}

static real_T c1_sqrt(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                      c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sqrt(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_b_error(SFc1_controller_v1InstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controller_v1InstanceStruct *chartInstance;
  chartInstance = (SFc1_controller_v1InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_c_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i0, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_controller_v1InstanceStruct *chartInstance;
  chartInstance = (SFc1_controller_v1InstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_d_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_controller_v1, const char_T
  *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_controller_v1), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_controller_v1);
  return c1_y;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_controller_v1InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sqrt(SFc1_controller_v1InstanceStruct *chartInstance, real_T
                      *c1_x)
{
  real_T c1_b_x;
  boolean_T c1_b0;
  boolean_T c1_p;
  c1_b_x = *c1_x;
  c1_b0 = (c1_b_x < 0.0);
  c1_p = c1_b0;
  if (c1_p) {
    c1_b_error(chartInstance);
  }

  *c1_x = muDoubleScalarSqrt(*c1_x);
}

static void init_dsm_address_info(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_controller_v1InstanceStruct
  *chartInstance)
{
  chartInstance->c1_x_drone = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c1_alpha_d = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_y_drone = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_z_drone = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_theta_d = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_phi_d = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_x_gs = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_y_gs = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_z_gs = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c1_theta_gs = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 8);
  chartInstance->c1_phi_gs = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 9);
  chartInstance->c1_alpha_gs = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_gamma_d = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_gamma_gs = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_opt_theta_gs = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_opt_theta_d = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_opt_phi_gs = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c1_opt_phi_d = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 8);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_controller_v1_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2300249506U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2998839726U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1008720302U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3778845868U);
}

mxArray* sf_c1_controller_v1_get_post_codegen_info(void);
mxArray *sf_c1_controller_v1_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("dwxDQp7FBIiN4WPAWIdphG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,10,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,8,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_controller_v1_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_controller_v1_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_controller_v1_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_controller_v1_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_controller_v1_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_controller_v1(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"alpha_d\",},{M[1],M[15],T\"alpha_gs\",},{M[1],M[16],T\"gamma_d\",},{M[1],M[17],T\"gamma_gs\",},{M[1],M[21],T\"opt_phi_d\",},{M[1],M[20],T\"opt_phi_gs\",},{M[1],M[19],T\"opt_theta_d\",},{M[1],M[18],T\"opt_theta_gs\",},{M[8],M[0],T\"is_active_c1_controller_v1\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_controller_v1_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_controller_v1InstanceStruct *chartInstance =
      (SFc1_controller_v1InstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _controller_v1MachineNumber_,
           1,
           1,
           1,
           0,
           18,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_controller_v1MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_controller_v1MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _controller_v1MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x_drone");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y_drone");
          _SFD_SET_DATA_PROPS(2,1,1,0,"z_drone");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta_d");
          _SFD_SET_DATA_PROPS(4,1,1,0,"phi_d");
          _SFD_SET_DATA_PROPS(5,1,1,0,"x_gs");
          _SFD_SET_DATA_PROPS(6,1,1,0,"y_gs");
          _SFD_SET_DATA_PROPS(7,1,1,0,"z_gs");
          _SFD_SET_DATA_PROPS(8,1,1,0,"theta_gs");
          _SFD_SET_DATA_PROPS(9,1,1,0,"phi_gs");
          _SFD_SET_DATA_PROPS(10,2,0,1,"alpha_d");
          _SFD_SET_DATA_PROPS(11,2,0,1,"alpha_gs");
          _SFD_SET_DATA_PROPS(12,2,0,1,"gamma_d");
          _SFD_SET_DATA_PROPS(13,2,0,1,"gamma_gs");
          _SFD_SET_DATA_PROPS(14,2,0,1,"opt_theta_gs");
          _SFD_SET_DATA_PROPS(15,2,0,1,"opt_theta_d");
          _SFD_SET_DATA_PROPS(16,2,0,1,"opt_phi_gs");
          _SFD_SET_DATA_PROPS(17,2,0,1,"opt_phi_d");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,7,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,3250);
        _SFD_CV_INIT_EML_IF(0,1,0,1998,2015,-1,2051);
        _SFD_CV_INIT_EML_IF(0,1,1,2052,2068,-1,2103);
        _SFD_CV_INIT_EML_IF(0,1,2,2154,2169,2198,2239);
        _SFD_CV_INIT_EML_IF(0,1,3,2240,2254,2281,2320);
        _SFD_CV_INIT_EML_IF(0,1,4,2374,2391,2482,2579);
        _SFD_CV_INIT_EML_IF(0,1,5,2748,2765,-1,2800);
        _SFD_CV_INIT_EML_IF(0,1,6,2801,2817,-1,2850);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,2001,2015,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,2055,2068,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,2157,2169,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,2243,2254,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,4,2377,2391,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,5,2751,2765,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,6,2804,2817,-1,4);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _controller_v1MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_controller_v1InstanceStruct *chartInstance =
      (SFc1_controller_v1InstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_x_drone);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c1_alpha_d);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c1_y_drone);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c1_z_drone);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_theta_d);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_phi_d);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c1_x_gs);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c1_y_gs);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c1_z_gs);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c1_theta_gs);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c1_phi_gs);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c1_alpha_gs);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c1_gamma_d);
        _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c1_gamma_gs);
        _SFD_SET_DATA_VALUE_PTR(14U, chartInstance->c1_opt_theta_gs);
        _SFD_SET_DATA_VALUE_PTR(15U, chartInstance->c1_opt_theta_d);
        _SFD_SET_DATA_VALUE_PTR(16U, chartInstance->c1_opt_phi_gs);
        _SFD_SET_DATA_VALUE_PTR(17U, chartInstance->c1_opt_phi_d);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s5n7gFykwGQmXcfZJsRF8IF";
}

static void sf_opaque_initialize_c1_controller_v1(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_controller_v1InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
    chartInstanceVar);
  initialize_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_controller_v1(void *chartInstanceVar)
{
  enable_c1_controller_v1((SFc1_controller_v1InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_controller_v1(void *chartInstanceVar)
{
  disable_c1_controller_v1((SFc1_controller_v1InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_controller_v1(void *chartInstanceVar)
{
  sf_gateway_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_controller_v1(SimStruct* S)
{
  return get_sim_state_c1_controller_v1((SFc1_controller_v1InstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_controller_v1(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_controller_v1(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_controller_v1InstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_controller_v1_optimization_info();
    }

    finalize_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_controller_v1((SFc1_controller_v1InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_controller_v1(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_controller_v1((SFc1_controller_v1InstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_controller_v1(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_controller_v1_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,10);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,8);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=8; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 10; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(916756912U));
  ssSetChecksum1(S,(1545427053U));
  ssSetChecksum2(S,(3059260009U));
  ssSetChecksum3(S,(3574823552U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_controller_v1(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_controller_v1(SimStruct *S)
{
  SFc1_controller_v1InstanceStruct *chartInstance;
  chartInstance = (SFc1_controller_v1InstanceStruct *)utMalloc(sizeof
    (SFc1_controller_v1InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_controller_v1InstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_controller_v1;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_controller_v1;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_controller_v1;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_controller_v1;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_controller_v1;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_controller_v1;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_controller_v1;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_controller_v1;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_controller_v1;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_controller_v1;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_controller_v1;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_controller_v1(chartInstance);
}

void c1_controller_v1_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_controller_v1(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_controller_v1(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_controller_v1(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_controller_v1_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
