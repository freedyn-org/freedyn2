# cmake -D DLLS="a.dll;b.dll" -D DEST=dir -P copy_dlls.cmake: copy the DLLs a program needs next
# to it (Windows; e.g. KLU). An empty list copies nothing.
foreach(dll IN LISTS DLLS)
  file(COPY "${dll}" DESTINATION "${DEST}")
endforeach()
