#add_subdirectory(pcg)		#deleted by wada
#add_subdirectory(dense)	#deleted by wada
#add_subdirectory(slam2d_linear)#deleted by wada
#add_subdirectory(structure_only)#deleted by wada

#deleted by wada
#if(CSPARSE_FOUND)
  #add_subdirectory(csparse)
#endif()

#deleted by wada
#if(CHOLMOD_FOUND)
  #add_subdirectory(cholmod)
#endif(CHOLMOD_FOUND)

# Sparse Module of Eigen is stable starting from 3.1
add_subdirectory(eigen)
