from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

# Cython 컴파일 설정
extensions = [
    Extension(
        "waypoint_node_manager.waypoint_node_manager",
        ["waypoint_node_manager/waypoint_node_manager.pyx"],
        include_dirs=[numpy.get_include()],
        extra_compile_args=["-O3"],  # 최적화 옵션
    )
]

setup(
    name="waypoint_node_manager",
    ext_modules=cythonize(
        extensions,
        compiler_directives={
            'language_level': "3",
            'boundscheck': True,  # 더 안전한 옵션으로 변경
            'wraparound': True,   # 더 안전한 옵션으로 변경
            'initializedcheck': True,  # 더 안전한 옵션으로 변경
            'nonecheck': True,    # 더 안전한 옵션으로 변경
        }
    ),
    zip_safe=False,
) 