/**
 * @file    mpc_qpOASES.h
 * @brief   Thin C adapter between the chassis MPC code and qpOASES' C API.
 *
 * The qpOASES C interface is NOT a pre-compiled library.  It is
 * qpOASES_wrapper.cpp + the C++ qpOASES sources.  This adapter hides that
 * C++ interface from the rest of the project and fixes real_t to float.
 *
 * Add mpc_qpOASES.c + qpOASES_wrapper.cpp to the Keil project, then use only
 * the functions below from plain C.
 */
#ifndef MPC_QPOASES_H
#define MPC_QPOASES_H

#ifdef __cplusplus
extern "C" {
#endif

/* nV = MPC_MAX_N * NU.  30 * 3 = 90 now; leave some headroom. */
#define MPC_QPOASES_MAX_NV 192

/**
 * @brief  Create the internal QProblemB object.
 * @param  nV number of optimisation variables (= horizon * nu)
 * @return 0 on success, negative on setup error.
 */
int MPC_QPOASES_Init(int nV);

/**
 * @brief  First solve (cold start) of
 *             min 0.5*x'Hx + g'x   s.t.  lb <= x <= ub
 * @param  H  nV*nV Hessian, column-major (symmetric, so row-major is also OK)
 * @param  g  nV gradient
 * @param  lb nV lower bounds
 * @param  ub nV upper bounds
 * @param  x  nV output: optimal primal solution
 * @return solver status: 0 = solved, 1 = max nWSR reached, 2 = infeasible,
 *         negative = API/setup error.
 */
int MPC_QPOASES_Solve(const float *H,
                      const float *g,
                      const float *lb,
                      const float *ub,
                      float *x);

/**
 * @brief  Warm start with a new gradient / new bounds (H stays unchanged).
 *         Call this once per control cycle after MPC_QPOASES_Solve().
 * @return same status encoding as MPC_QPOASES_Solve().
 */
int MPC_QPOASES_Hotstart(const float *g,
                         const float *lb,
                         const float *ub,
                         float *x);

/** Last solver status (0 solved, 1 max nWSR, 2 infeasible). */
int MPC_QPOASES_GetLastStatus(void);

/** Number of working-set recalculations used by the last call. */
int MPC_QPOASES_GetLastNWSR(void);

/** Objective value of the last solution. */
float MPC_QPOASES_GetLastObj(void);

/** Release the internal qpOASES object. */
void MPC_QPOASES_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* MPC_QPOASES_H */
