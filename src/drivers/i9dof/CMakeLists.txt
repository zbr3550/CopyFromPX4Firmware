
px4_add_module(
	MODULE drivers__i9dof
	MAIN i9dof
	STACK 2000
	COMPILE_FLAGS
		-Weffc++
		-Os
	SRCS
		i9dof.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 