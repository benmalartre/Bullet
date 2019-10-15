# Bullet makefile
BULLET_PATH = /Users/benmalartre/Documents/RnD/bullet3

# Includes
BULLET_INCLUDES = -I$(BULLET_PATH)/src
BULLET_LIBRARIES = -L$(BULLET_PATH)/bin

#CFLAGS += -std=c++11  -DNDEBUG -c $(BULLET_INCLUDES) -static -mmacosx-version-min=10.7\

# settings
#
CFLAGS = -DPLATFORM_DARWIN \
	 -DCC_GNU_ \
	 -DOSMac_ \
	 -DOSMacOSX_ \
	 -DOSMac_MachO_ \
	 -DPB_MACOS
	 -O3 \
	 -D_LANGUAGE_C_PLUS_PLUS\
	 -mmacosx-version-min=10.9\
	 -arch x86_64\
	 -fPIC\
	 -DNDEBUG\
	 -static

C++FLAGS = $(CFLAGS) \
	   -std=c++11 \
	   -stdlib=libc++ \
	   -fno-gnu-keywords \
	   -fpascal-strings  \
	   -Wno-deprecated \
	   -Wpointer-arith \
	   -Wwrite-strings \
	   -Wno-long-long

# iphone SDK
#CFLAGS += -arch arm64 -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS10.2.sdk

# Apple SDK
CFLAGS += -arch x86_64 -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk

#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/bin/#libBullet3Collision_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/#libBullet3Common_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/arm64/bin/#libBullet3Dynamics_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/arm64/bin/#libBullet3Geometry_gmake_x64_release.a

LDFLAGS += -static $(LIBS)

# Rules
#./configure armv7 --build x86_64 --host=arm-apple-darwin10 --target=aarch64-apple-darwin #CC=/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang EXTRA_CFLAGS='-arch arm64 -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS10.2.sdk' EXTRA_LDFLAGS='-arch arm64'


all: CAPI.o bullet.a

CAPI.o: capi.cpp capi.h
	clang++ $(BULLET_INCLUDES) -c $(C++FLAGS) -Wall	-mmacosx-version-min=10.7 -g capi.cpp

bullet.a:  CAPI.o
	ar rcs $@ $^ 

clean:
	rm -rf *.o *.gah