#	$NetBSD$
#
# audioctl(1) for the ESP32-S31 root file system: usr.bin/audio/ctl
# with its private libaudio compiled in.
#

.include <bsd.own.mk>

SRCDIR=		${NETBSDSRCDIR}/usr.bin/audio

PROG=		audioctl
NOMAN=		# defined
SRCS=		ctl.c audio.c decode.c wav.c sun.c
CPPFLAGS+=	-I${SRCDIR}/common

.include <bsd.prog.mk>

.PATH:		${SRCDIR}/ctl ${SRCDIR}/common
