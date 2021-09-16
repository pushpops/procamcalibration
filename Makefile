# (1)コンパイラ
CC  = g++
# (2)コンパイルオプション
# CFLAGS    =
# (3)実行ファイル名
TARGET  = campro
# (4)コンパイル対象のソースコード
SRCS    += main.cpp \
		   calibration.cpp

#HEADERS += pso.h

# (5)オブジェクトファイル名
OBJS    = $(SRCS:.cpp=.o)
 
# (6)インクルードファイルのあるディレクトリパス
#INCDIR += 

# (7)ライブラリファイルのあるディレクトリパス

LIBDIR += -I/usr/local/Cellar/eigen/3.3.8_1/include/
#LIBDIR += /Users/maru/Downloads/boost_1_76_0
#LIBDIR += /usr/local/Cellar/boost/1.75.0_2/include/boost/
LIBDIR += -I/usr/local/Cellar/opencv@3/3.4.13_2/include/opencv2
LIBDIR += -I/usr/local/Cellar/opencv@3/3.4.13_2/include/opencv

# (8)追加するライブラリファイル
LIBS += -L/usr/local/Cellar/eigen/3.3.8_1/include/eigen3/Eigen
LIBS += -L/usr/local/Cellar/opencv@3/3.4.13_2/lib -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_dnn -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
#LIBS += -L/Users/maru/Downloads/boost_1_76_0/stage/lib -lboost_system -lboost_filesystem
#LIBS += -L/Users/maru/Downloads/boost_1_76_0 -lboost_system -lboost_filesystem
#LIBS += -L/usr/local/Cellar/boost/1.75.0_2/include/boost -lboost_system -lboost_filesystem
#LIBS += -L/usr/local/Cellar/boost/1.75.0_2/include/boost/

# (9)ターゲットファイル生成
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBDIR) $(LIBS)
	
# (10)オブジェクトファイル生成
$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCS)

# (11)"make all"で make cleanとmakeを同時に実施。
all: clean $(OBJS) $(TARGET)
# (12).oファイル、実行ファイル、.dファイルを削除
clean:
	-rm -f $(OBJS) $(TARGET) *.d