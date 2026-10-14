#	$NetBSD$
#
# ftp(1) for the ESP32-S31 root file system: usr.bin/ftp without TLS (the
# root file system has no OpenSSL), but with line editing and IPv6.
#

.include <bsd.own.mk>

SRCDIR=		${NETBSDSRCDIR}/usr.bin/ftp

PROG=		ftp
NOMAN=		# defined
SRCS=		cmds.c cmdtab.c complete.c domacro.c fetch.c ftp.c main.c \
		progressbar.c ruserpass.c uri.c util.c ssl.c
CPPFLAGS+=	-I${SRCDIR} -DINET6
DPADD+=		${LIBEDIT} ${LIBTERMINFO}
LDADD+=		-ledit -lterminfo

.include <bsd.prog.mk>

.PATH:		${SRCDIR}
