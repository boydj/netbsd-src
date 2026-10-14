/*	$NetBSD: fenv.h,v 1.5 2024/05/12 20:04:12 riastradh Exp $	*/

/*
 * Based on ieeefp.h written by J.T. Conklin, Apr 28, 1995
 * Public domain.
 */

#ifndef _RISCV_FENV_H_
#define _RISCV_FENV_H_

#include <sys/cdefs.h>

typedef int fenv_t;		/* FPSCR */
typedef int fexcept_t;

#define	FE_INEXACT	((int)__BIT(0))	/* Result inexact */
#define	FE_UNDERFLOW	((int)__BIT(1))	/* Result underflowed */
#define	FE_OVERFLOW	((int)__BIT(2))	/* Result overflowed */
#define	FE_DIVBYZERO	((int)__BIT(3))	/* divide-by-zero */
#define	FE_INVALID	((int)__BIT(4))	/* Result invalid */

#define	FE_ALL_EXCEPT	\
    (FE_INEXACT | FE_UNDERFLOW | FE_OVERFLOW | FE_DIVBYZERO | FE_INVALID)

#define	FE_TONEAREST	0	/* round to nearest representable number */
#define	FE_TOWARDZERO	1	/* round to zero (truncate) */
#define	FE_DOWNWARD	2	/* round toward negative infinity */
#define	FE_UPWARD	3	/* round toward positive infinity */

#ifdef _SOFT_FLOAT
/*
 * Soft-float (e.g. RV32IMAC, ilp32): the environment is kept in the fcsr
 * layout, plus the (software only) exception mask.
 */
#define	__FENV_FLAGS	__BITS(4, 0)	/* fcsr.fflags */
#define	__FENV_ROUND	__BITS(7, 5)	/* fcsr.frm */
#define	__FENV_MASK	__BITS(12, 8)	/* software trap enables */

#define	__FENV_GET_FLAGS(__envp)	__SHIFTOUT(*(__envp), __FENV_FLAGS)
#define	__FENV_GET_MASK(__envp)		__SHIFTOUT(*(__envp), __FENV_MASK)
#define	__FENV_GET_ROUND(__envp)	__SHIFTOUT(*(__envp), __FENV_ROUND)
#define	__FENV_SET_FLAGS(__envp, __val) \
	(*(__envp) = (*(__envp) & ~__FENV_FLAGS) | \
	    __SHIFTIN((__val), __FENV_FLAGS))
#define	__FENV_SET_MASK(__envp, __val) \
	(*(__envp) = (*(__envp) & ~__FENV_MASK) | \
	    __SHIFTIN((__val), __FENV_MASK))
#define	__FENV_SET_ROUND(__envp, __val) \
	(*(__envp) = (*(__envp) & ~__FENV_ROUND) | \
	    __SHIFTIN((__val), __FENV_ROUND))

#define	__HAVE_FENV_SOFTFLOAT_DEFS
#endif /* _SOFT_FLOAT */

__BEGIN_DECLS

/* Default floating-point environment */
extern const fenv_t	__fe_dfl_env;
#define FE_DFL_ENV	(&__fe_dfl_env)

__END_DECLS

#endif /* _RISCV_FENV_H_ */
