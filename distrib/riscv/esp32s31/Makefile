#	$NetBSD$
#
# Root file system for the Espressif ESP32-S31 kernel (sys/arch/riscv/conf/
# ESP32S31): a crunched userland, written to SPI flash at offset 0x800000
# and mounted read-only from there (see espxiproot).  At most 8MB.
#

.include <bsd.own.mk>
.include "${NETBSDSRCDIR}/distrib/common/Makefile.distrib"

IMAGE=		esp32s31-root.fs
IMAGESIZE=	6144k
MAKEFS_FLAGS+=	-o density=4096 -f 600

WARNS=		1
DBG=		-Os

CRUNCHBIN=	esp32s31bin
# Full-featured programs (line editing in sh, multi-user init, ...)
SMALLPROG=	0
LISTS=		${.CURDIR}/list
MTREECONF=	${.CURDIR}/mtree.conf
IMAGEENDIAN=	le

PARSELISTENV+=	TOOL_PWD_MKDB=${TOOL_PWD_MKDB:Q}

# login(1) without PAM
CRUNCHENV+=	USE_PAM=no USE_SKEY=no USE_KERBEROS=no USE_YP=no

IMAGEDEPENDS=	${CRUNCHBIN} \
		${.CURDIR}/etc/rc ${.CURDIR}/etc/ttys ${.CURDIR}/etc/gettytab \
		${.CURDIR}/etc/master.passwd ${.CURDIR}/etc/motd \
		${.CURDIR}/etc/profile ${.CURDIR}/etc/fstab \
		${NETBSDSRCDIR}/etc/group

MAKEDEVTARGETS=	std md0 ptm

# Use stubs to eliminate some large stuff from libc
HACKSRC=	${DISTRIBDIR}/utils/libhack
.include	"${HACKSRC}/Makefile.inc"
${CRUNCHBIN}:	libhack.o

.include "${DISTRIBDIR}/common/Makefile.crunch"
.include "${DISTRIBDIR}/common/Makefile.makedev"
.include "${DISTRIBDIR}/common/Makefile.image"

release:

.include <bsd.prog.mk>
