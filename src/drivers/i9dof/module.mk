
#
# Makefile to build the I9DOF driver.
#

MODULE_COMMAND	= i9dof

SRCS		= i9dof.cpp

MODULE_STACKSIZE	= 2000

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os