import ctypes
import numpy

cbc = ctypes.cdll.LoadLibrary("./build/libCBC.so")

camera_hight = 2.350
fx = 424.977
fy = 424.977
cx = 420.337
cy = 241.23
save_flag = 1
cbc.cbcSetConfig.argtypes = (ctypes.c_double, ctypes.c_double,
                             ctypes.c_double, ctypes.c_double,
                             ctypes.c_double, ctypes.c_int)
cbc.cbcSetConfig(camera_hight,  fx,  fy,  cx,  cy,  save_flag)

data = numpy.loadtxt("./data/dp.txt")
img = data.astype(numpy.uint16)
tmp = numpy.asarray(img)
data_ptr = tmp.ctypes.data_as(ctypes.c_char_p)

H_ptr = (ctypes.c_double*1)()
VL_ptr = (ctypes.c_double*1)()
VR_ptr = (ctypes.c_double*1)()
H_ptr[0] = 0
VL_ptr[0] = 0
VR_ptr[0] = 0
re = cbc.cbcGetValue(img.shape[1], img.shape[0],
                     data_ptr, H_ptr, VL_ptr, VR_ptr)
print('==========')
print(re)
print(H_ptr[0])
print(VL_ptr[0])
print(VR_ptr[0])

area_up_ptr = (ctypes.c_double*2)()
area_down_ptr = (ctypes.c_double*2)()
re = cbc.cbcGetArea(area_up_ptr, area_down_ptr)
print('area==========')
print(re)
print("up", area_up_ptr[0], area_up_ptr[1])
print("down", area_down_ptr[0], area_down_ptr[1])


point1_ptr = (ctypes.c_double*2)()
point2_ptr = (ctypes.c_double*2)()
point3_ptr = (ctypes.c_double*2)()
point5_ptr = (ctypes.c_double*2)()
point7_ptr = (ctypes.c_double*2)()
re = cbc.cbcGetInterestPoint(
    point1_ptr, point2_ptr, point3_ptr, point5_ptr, point7_ptr)
print('point==========')
print(re)
print("num1", point1_ptr[0], point1_ptr[1])
print("num2", point2_ptr[0], point2_ptr[1])
print("num3", point3_ptr[0], point3_ptr[1])
print("num5", point5_ptr[0], point5_ptr[1])
print("num7", point7_ptr[0], point7_ptr[1])
