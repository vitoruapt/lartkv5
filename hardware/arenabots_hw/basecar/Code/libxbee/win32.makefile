#-- uncomment this to enable debugging
#DEBUG:=/Zi /DDEBUG

#-- you may need to edit these lines if your installation is different
VCPath:=C:\Program Files (x86)\Microsoft Visual Studio 9.0\VC
SDKPath:=C:\Program Files\Microsoft SDKs\Windows\v6.0A


###### YOU SHOULD NOT CHANGE BELOW THIS LINE ######
SHELL:=cmd

SRCS:=api.c

CC:="${VCPath}\bin\cl.exe"
LINK:="${VCPath}\bin\link.exe"
RC:="${SDKPath}\bin\rc.exe"

.PHONY: all new clean 

all: .\lib\libxbee.dll

new: clean all

clean:
	-rmdir /Q /S lib
	-rmdir /Q /S obj

.\lib\libxbee.dll: .\lib .\obj\api.obj .\obj\win32.res
	${LINK} /nologo /DLL /MAP:lib\libxbee.map /DEF:xsys\win32.def \
		"/LIBPATH:${SDKPath}\Lib" "/LIBPATH:${VCPath}\lib" \
		/OUT:.\lib\libxbee.dll .\obj\api.obj .\obj\win32.res

.\obj\api.obj: .\obj api.c api.h xbee.h
	${CC} ${DEBUG} /nologo "/I${SDKPath}\Include" "/I${VCPath}\include" /MT /RTCs /Gz /c /Fd.\lib\libxbee.pdb /Fo.\obj\api.obj ${SRCS}

.\obj\win32.res: .\xsys\win32.rc
	${RC} "/I${SDKPath}\Include" "/I${VCPath}\include" /n /fo.\obj\win32.res .\xsys\win32.rc 

.\obj:
	mkdir obj

.\lib:
	mkdir lib  
