#
# Makefile:
#	TWB Trigger Projekt. 
#	https://github.com/kevinp1993/TWB_Trigger
#
################################################################################
#################################################################################

ifneq ($V,1)
Q ?= @
endif

#DEBUG	= -g -O0
DEBUG	= -O3
CC	= g++
INCLUDE	= -I/usr/local/include $(CURDIR)
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -o

LDFLAGS	= -L/usr/local/lib
DEPS = amiroArucoGlobal.hpp
#LDLIBS    = 

# Should not alter anything below this line
###############################################################################

SRC	=	jevoisAruco_AmiroDist 				\

OBJ	=	$(SRC:.cpp=.o)

BINS	=	$(SRC:.cpp=)

all:	$(BINS)


.c.o:
	$Q echo [CC] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@

clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
