import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
#from ybUtils.YbKey import YbKey
#from ybUtils.YbUart import YbUart
from machine import Pin

from machine import UART

from machine import FPIOA


from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import nncase_runtime as nn
import aidemo

#key = YbKey()
#uart = YbUart(baudrate=115200)
#最下面的tx
# 显示参数 / Display parameters
DISPLAY_WIDTH = 640    # LCD显示宽度 / LCD display width
DISPLAY_HEIGHT = 480   # LCD显示高度 / LCD display height

BLACK_THRESHOLD = (0, 50)  # 推荐范围：0-50（值越小检测越严格的黑色）
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

from machine import UART

from machine import FPIOA


# 定义色块检测参数
MIN_PIXELS = 300     # 最小像素数（过滤噪声）
MAX_PIXELS = 75000   # 最大像素数（过滤大面积区域）
AREA_THRESHOLD = 50  # 区域面积阈值（可选）
clock = time.clock()
max_density=0.4   #限制色块密度
max_solidity=0.5  #控制实心度
max_convexity=0.65
min_convexity=0.45   #限制凸度
min_area=5000
max_area=73000
WIDTH = 640
HEIGHT = 480


def generate_uart_message(x,y):
    # 确保整数在有效范围内 (0-255)


    int1 = 1
    int2 = 1


    if x < 0:
        int1 = 0
        x = -x
    if y < 0:
        int2 = 0
        y = -y

    x = max(0, min(255, x))
    y = max(0, min(255, y))

    message = bytes([0x7A, int1, x, int2, y,1, 0x7B])

    return bytes(message)

def line_intersection(line1, line2):
    """
    计算两条直线的交点坐标
    """
    (x1, y1, x2, y2) = line1
    (x3, y3,x4, y4) = line2

    # 计算分母（判断是否平行）
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        # 分母为0，两直线平行或重合，无唯一交点
        return (abs(x2-x1)//2,abs(y2-y1)//2)

    # 计算分子
    t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    s_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)

    t = t_num / den
    s = s_num / den

    # 计算交点坐标
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)

    return (x, y)

def fabs(x):
    if x < 0:
        return -x




def init_sensor():
    """初始化摄像头 / Initialize camera sensor"""
    sensor = Sensor(id = 2)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT,chn=CAM_CHN_ID_0)
    sensor.set_pixformat(sensor.GRAYSCALE,chn=CAM_CHN_ID_0)  # 设置为灰度模式

    sensor.set_framesize(width = WIDTH, height = HEIGHT,chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_1)
    return sensor



def init_display():
    """初始化显示 / Initialize display"""
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

# 前段先用AI检测
def main():
    try:

        # Define the input size for the RGB888P video frames
        rgb888p_size = [640, 480]

        # Set root directory path for model and config
        root_path = "/sdcard/mp_deployment_source/"

        # Load deployment configuration
        deploy_conf = read_json(root_path + "/deploy_config.json")
        kmodel_path = root_path + deploy_conf["kmodel_path"]              # KModel path
        labels = deploy_conf["categories"]                                # Label list
        confidence_threshold = deploy_conf["confidence_threshold"]        # Confidence threshold
        nms_threshold = deploy_conf["nms_threshold"]                      # NMS threshold
        model_input_size = deploy_conf["img_size"]                        # Model input size
        nms_option = deploy_conf["nms_option"]                            # NMS strategy
        model_type = deploy_conf["model_type"]                            # Detection model type
        anchors = []
        if model_type == "AnchorBaseDet":
            anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

        # Inference configuration
        inference_mode = "video"                                          # Inference mode: 'video'
        display_size = [640,480]
        debug_mode = 0                                                    # Debug mode flag
        target_det = DetectionApp(inference_mode,kmodel_path,labels,model_input_size,anchors,model_type,confidence_threshold,nms_threshold,rgb888p_size,display_size,debug_mode=debug_mode)
        target_det.config_preprocess()


        # 初始化设备 / Initialize devices
        sensor = init_sensor()
        init_display()
        sensor.run()
        #uart.write(bytearray([0x55, 0xaa, 0xff, int(222)&0xFF, int(333)&0xFF, 0xfa]))
        uart.init(baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        clock = time.clock()

        error_effect_flag = False
        LED_R = Pin(62, Pin.OUT, pull=Pin.PULL_NONE, drive=7) # 红灯
        LED_B = Pin(63, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
        LED_G = Pin(20, Pin.OUT, pull=Pin.PULL_NONE, drive=7) # 绿灯
        Laser = Pin(32, Pin.OUT, pull=Pin.PULL_UP, drive=15) # 绿灯

        LED_R.low()
        LED_G.high()
        LED_B.low()
        Laser.low()



        usr=Pin(53,Pin.IN,Pin.PULL_DOWN) #按键启动

#        Laser.high()




        while 1:
            if usr.value()==1:
                break
        LED_R.low()
        LED_G.low()
        LED_B.high()

        message = bytes([0x7A,1,1,1,1,1,0x7B])
        uart.write(message)
        while True:
            os.exitpoint()
            img_ai = sensor.snapshot(chn=CAM_CHN_ID_1)
            img_ai=img_ai.to_numpy_ref()
            res = target_det.run(img_ai)
            flag = False
            if len(res["boxes"]):
                for i in range(len(res["boxes"])):
                    if(res["scores"][i] > 0.84):
                        flag = True
                    if res["scores"][i] > 0.75:
                        x=int(res["boxes"][i][0] )
                        y=int(res["boxes"][i][1] )
                        w = int(float(res["boxes"][i][2] - res["boxes"][i][0]))
                        h = int(float(res["boxes"][i][3] - res["boxes"][i][1]) )
                        print(res["scores"][i])
                        horizon_error = (int)(339 - (x + w/2));
                        vertical_error = (int)(y+h/2 - 260);
                        message = generate_uart_message(horizon_error,vertical_error)
                        uart.write(message)
                    else:
                        message = bytes([0x7A,0,0,0,0,0x7B])
                        uart.write(message)
            else:
                message = bytes([0x7A,0,0,0,0,0,0x7B])
                uart.write(message)

            if flag:
                break

        LED_G.low()
        LED_B.low()
        LED_R.high()

        yes_counter = 0


        while True:
            clock.tick()
            os.exitpoint()
            # 捕获图像 / Capture image
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img.draw_cross(320,240)
            error_effect_flag = False

            img_ai = sensor.snapshot(chn=CAM_CHN_ID_1)
            img_ai=img_ai.to_numpy_ref()
            res = target_det.run(img_ai)
            ai_flag = False
            x = 0
            y = 0
            h = 0
            w = 0
            if len(res["boxes"]):
                for i in range(len(res["boxes"])):
                    if res["scores"][i] > 0.6:
                        x=int(res["boxes"][i][0] )
                        y=int(res["boxes"][i][1] )
                        w = int(float(res["boxes"][i][2] - res["boxes"][i][0]))
                        h = int(float(res["boxes"][i][3] - res["boxes"][i][1]) )
                        print(res["scores"][i])
                        ai_flag = True

            # 查找符合阈值的色块
            blobs = img.find_blobs([BLACK_THRESHOLD],
                                  pixels_threshold=MIN_PIXELS,
                                  area_threshold=AREA_THRESHOLD,
                                  x_stride=1,
                                  y_stride=1,
                                  margin=15,
                                  )  # 合并相邻色块

            if blobs:
                valid_blobs = []
                for blob in blobs:
                    if blob.density()<max_density:
                        if blob.area()>min_area :
                            img.draw_rectangle(blob.rect(), color=(255,0,0))
                            #print(blob.density(),blob.solidity(),blob.convexity())#像素数除以其边界框区域,使用最小区域旋转矩形与边界矩形来测量密度,对象的凸度
                            ##0.152698 0.167296 0.423312
                            # valid_blobs.append(blob)
                            if blob.solidity()<max_solidity:
                                if blob.convexity()<max_convexity:
                                    valid_blobs.append(blob)
                #print("********************************************************************************")


            if valid_blobs:
                for min_density_blob in valid_blobs:

#                    img.draw_line(min_density_blob.major_axis_line())
#                    img.draw_line(min_density_blob.minor_axis_line())
                    intersection=line_intersection(min_density_blob.major_axis_line(), min_density_blob.minor_axis_line())

                    lx1=abs(min_density_blob.major_axis_line()[0]-min_density_blob.major_axis_line()[2])
                    ly1=abs(min_density_blob.major_axis_line()[1]-min_density_blob.major_axis_line()[3])
                    l1=lx1**2+ly1**2
                    l1=l1**(1/2)
                    lx2=abs(min_density_blob.minor_axis_line()[0]-min_density_blob.minor_axis_line()[2])
                    ly2=abs(min_density_blob.minor_axis_line()[1]-min_density_blob.minor_axis_line()[3])
                    l2=lx2**2+ly2**2
                    l2=l2**(1/2)
                    #print(l2/l1)
#                    if (l2/l1>0.65):
                    if ((l2/l1>0.65) and  int(intersection[0]) > x and int(intersection[0]) < x+w and int(intersection[1]) > y and int(intersection[1]) < y+h) or l2/l1>0.72:#可调
#                        img.draw_rectangle(min_density_blob.rect(), color=(255,0,0))
#                        img.draw_cross(min_density_blob.min_corners()[0][0], min_density_blob.min_corners()[0][1])
#                        img.draw_cross(min_density_blob.min_corners()[1][0], min_density_blob.min_corners()[1][1])
#                        img.draw_cross(min_density_blob.min_corners()[2][0], min_density_blob.min_corners()[2][1])
#                        img.draw_cross(min_density_blob.min_corners()[3][0], min_density_blob.min_corners()[3][1])
#                        img.draw_circle(min_density_blob.enclosing_circle(),color=(255,255,0), thickness=2, fill=False)
                        img.draw_cross(int(intersection[0]),int(intersection[1]), color=(255,0,0))
                        img.draw_cross(int(intersection[0]),int(intersection[1]), color=(255,0,0))
                        img.draw_cross(x,y, color=(255,0,0))
                        img.draw_cross(x+w,y+h, color=(255,0,0))
                        print(intersection)#靶心坐标
                        horizon_error = (int)(337 - intersection[0]);
                        vertical_error = (int)(intersection[1] - 256);
                        print("横向误差",horizon_error)
                        print("纵向误差",vertical_error)
                        if(horizon_error < 15 and horizon_error > -15 and
                           vertical_error < 15 and vertical_error > -15 and yes_counter < 5):
                            yes_counter+=1
                        if yes_counter >= 5:
                            Laser.high()
                        error_effect_flag = True
                        message = generate_uart_message(horizon_error,vertical_error)
                        uart.write(message)
                    elif ai_flag:
                        horizon_error = (int)(337 - (x + w/2));
                        vertical_error = (int)(y+h/2 - 256);
                        message = generate_uart_message(horizon_error,vertical_error)
                        print("横向误差",horizon_error)
                        print("纵向误差",vertical_error)
                        uart.write(message)
                    else:
                        message = bytes([0x7A,0,0,0,0,0,0x7B])
                        uart.write(message)



            fps_text = f'FPS: {clock.fps():.3f}'
            img.draw_string_advanced(0, 0, 30, fps_text, color=(255, 255, 255))
            Display.show_image(img)

            # 显示图像并打印FPS / Display image and print FPS

            print(clock.fps())

    except KeyboardInterrupt as e:
        print("用户中断 / User interrupted: ", e)
    except Exception as e:
        print(f"发生错误 / Error occurred: {e}")
    finally:
        # 清理资源 / Cleanup resources
        if 'sensor' in locals() and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
