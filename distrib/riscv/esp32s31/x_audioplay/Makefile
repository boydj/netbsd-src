#	$NetBSD$
#
# audioplay(1) for the ESP32-S31 root file system: usr.bin/audio/play
# with its private libaudio compiled in.
#

.include <bsd.own.mk>

SRCDIR=		${NETBSDSRCDIR}/usr.bin/audio

PROG=		audioplay
NOMAN=		# defined
SRCS=		play.c audio.c decode.c wav.c sun.c
CPPFLAGS+=	-I${SRCDIR}/common
DPADD+=		${LIBUTIL}
LDADD+=		-lutil

.include <bsd.prog.mk>

.PATH:		${SRCDIR}/play ${SRCDIR}/common
