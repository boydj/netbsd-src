#	$NetBSD$
#
# fptest: FPU and FP context switching test (see fptest.c).  Built with the
# F extension; the ilp32 ABI keeps it compatible with the soft-float libc.
#

PROG=	fptest
MAN=
CFLAGS+=	-march=rv32imafc_zicsr_zifencei -mabi=ilp32

.include <bsd.prog.mk>
