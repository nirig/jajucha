from jajuchaUtil import *
#2018년 12월 21일 

################################ set cascade ################################
trafficlight_cascade = cv2.CascadeClassifier('haar.xml')
#############################################################################


def findTrafficLight(image):
    # ROI 설정
    height, width = image.shape[:2]
    ROI_image = image[:120, :]
    light = False # Red light
    
    # 학습 모델을 사용하여 신호등 위치 추정
    # trafficlight = trafficlight_cascade.detectMultiScale(ROI_gray, 1.3, 5)
    trafficlight = trafficlight_cascade.detectMultiScale(ROI_image, 1.1, 5)
    cv2.imshow('roi', ROI_image)

    for (x,y,w,h) in trafficlight:
        # 신호등이 30cm 이상 밖에 있다고 판단하고 그대로 함수 종료
        if w < 35:
            break

        # 신호등이 30cm 이내 거리에 있다고 판단
        ROI = ROI_image[y:y+h, x:x+w]        
        ROI_X = ROI.shape[1]
        ROI_Y = ROI.shape[0]

        light_position = -1
        
        # 신호등 위치에서 밝기를 기준으로 왼쪽과 오른쪽 중 어느곳에 불이 들어와 있는지 판단
        for i in range(ROI_X):
            for j in range(ROI_Y):
                if ROI_image.item(y,x) > 150: # 밝기 비교
                    light_position = i
        
        # 오른쪽에 불이 들어와 있다면 green을 리턴 (기본값 False)
        if light_position < ROI_X/2 - 4:
            light = True
        
        return (x, y, w, h, light)
    
    # 발견된 신호등이 없으면 그냥 False 리턴
    # False is red / True is green
    return False

# def stop(image):
#     height, width = image.shape[:2]
#     stop = True
#     status = "None"
#     for y in range(100, 240):
#         for x in range(300, 160, -1):
#             B = image.item(y,x,0)
#             G = image.item(y,x,1)
#             R = image.item(y,x,2)
#             if 50 < B < 70 and 45 < G < 65 and 85 < R < 110:
#                 print(B, G, R)
#                 print(x,",",y)
#                 status = 'Stop'
#                 stop = False
#                 break
#         if stop == False:
#             break
#     return status
 
def autoDrive_algorithm(original_img, canny_img, gray_img, points, LiDAR, prevComm, status, light):
    height, width = canny_img.shape[:2]
    debug = True
    # debug = False
    command = prevComm
    result = findTrafficLight(gray_img)
    
    if result != False:
        x = result[0]
        y = result[1]
        w = result[2]
        h = result[3]
        light = result[4]
        cv2.rectangle(original_img,(x,y),(x+w,y+h),(0,0,255),2)

    H1LD = points['H1LD']
    H1RD = points['H1RD']
    H2LD = points['H2LD']
    H2RD = points['H2RD']
    H3LD = points['H3LD']
    H3RD = points['H3RD']
    H4LD = points['H4LD']
    H4RD = points['H4RD']
    H5LD = points['H5LD']
    H5RD = points['H5RD']
    H6LD = points['H6LD']
    H6RD = points['H6RD']
 
    V1D = points['V1D']
    V2D = points['V2D']
    V3D = points['V3D']
    V4D = points['V4D']
    V5D = points['V5D']
    V6D = points['V6D']
    V7D = points['V7D']
 
    la = [0, 0, 0, 0, 0, 0]
    ra = [320, 320, 320, 320, 320, 320]
    ea = [0, 0, 0, 0, 0, 0, 0]
    la[0] = H1LD
    la[1] = H2LD
    la[2] = H3LD
    la[3] = H4LD
    la[4] = H5LD
    la[5] = H6LD
    ra[0] = H1RD
    ra[1] = H2RD
    ra[2] = H3RD
    ra[3] = H4RD
    ra[4] = H5RD
    ra[5] = H6RD
    ea[0] = V1D
    ea[1] = V2D
    ea[2] = V3D
    ea[3] = V4D
    ea[4] = V5D
    ea[5] = V6D
    ea[6] = V7D

    if light:
        print('Traffic light is RED')
        command = 'S0150E'
        light = False

    #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    # 라이다 센서로 정지 표시를 감지 (장애물과의 거리를 측정, 거리가 35cm 이하인 경우 거리를 출력하며 자동차를 멈춤
    #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
    elif  LiDAR > 0 and LiDAR < 350:  #LiDAR sensor
            print('Obstacle Detected at %dmm' % LiDAR)
            command = 'S0150E' 

    else:
        # 가운데의 차선 중 하나 이상의 차선이 매우 가까운 경우 (보통 코너를 돌고 있을 때 차선이 매우 가까이 감지됨. 즉, 코너를 돌고 있는 경우를 의미함.)
        if V3D > 185 or V4D > 178 or V5D > 185:
            # 이전 커맨드가 회전 커맨드인 경우
            if prevComm == 'S1110E' or prevComm == 'S1190E' or prevComm == 'S1135E' or prevComm == 'S1165E':
                # 이전 커맨드가 왼쪽으로 회전하고 있던 경우
                if prevComm == 'S1110E' or prevComm == 'S1135E':
                    # 차선이 매우 가까운 경우 크게 회전을, 아닌 경우 적게 회전을 하는 명령을 내린다.
                    if V3D > 185: command = 'S1110E'
                    else: command = 'S1135E'
                # 이전 커맨드가 오른쪽으로 회전하고 있던 경우
                else: 
                    # 차선이 매우 가까운 경우 크게 회전을, 아닌 경우 적게 회전을 하는 명령을 내린다.
                    if V5D > 185: command = 'S1190E'
                    else: command = 'S1165E'

            # 이전 커맨드가 직진 또는 중앙 유지인 경우(가운데의 세 차선 중 하나 이상의 차선이 가까우나, 아직 커브를 하지 않은 경우)
            else:
                # 감지되지 않는 중앙 차선의 개수를 센다
                temp = 0
                for i in ea:
                    if i == 0: temp += 1

                # 감지되지 않은 중앙 차선이 세 개 이하인 경우
                if temp <= 3:
                    # 감지되지 않는 오른쪽 차선의 개수를 센다
                    temp1 = 0
                    for i in ra:
                        if i == 320: temp1 += 1

                    # 감지되지 않는 왼쪽 차선의 개수를 센다
                    temp2 = 0
                    for i in la:
                        if i == 0: temp2 += 1
                    
                    # 오른쪽 차선이 더 적게 감지가 된 경우 오른쪽으로 회전, 왼쪽 차선이 더 적게 감지가 된 경우 왼쪽으로 회전
                    if temp1 > temp2: command = 'S1190E'
                    elif temp1 < temp2: command = 'S1110E'

                # 중앙 차선이 4개 이상 감지되지 않은 경우
                else:
                    # 좌, 우 차선이 감지가 되나, 중앙을 중심으로 많이 벗어난 경우
                    if (abs((160-H2LD)-(H2RD-160)) > 10 and H2LD != 0 and H2RD != 320) or (abs((160-H4LD)-(H4RD-160)) > 30 and H4LD != 0 and H4RD != 320):
                        # 오른쪽으로 많이 기울어 진 경우 왼쪽으로 조금 회전, 왼쪽으로 많이 기울어 진 경우 오른쪽으로 조금 회전
                        if ((160-H2LD)-(H2RD-160) > 0 and H2LD != 0 and H2RD != 320) or ((160-H4LD)-(H4RD-160) > 0 and H4LD != 0 and H4RD != 320): command = 'S1140E'
                        else: command = 'S1160E'
                    # 차가 중심에서 크게 벗어나지 않은 경우 직진
                    else:
                        command = 'S1150E'
        # 가운데 중심 3개의 차선이 멀리 있거나, 감지되지 않는 경우
        else: 
            # 가장 멀리 있는 차선과의 거리를 구한다. 300은 임의의 큰 값, 가장 멀리있는 차선이 감지된 경우 해당 거리로 교체
            temp = 300
            for i in ea: 
                if i < temp: temp = i

            # 가장 멀리 있는 차선이 특정 값보다 더 멀리 있는 경우
            if temp < 185:
                # 좌, 우 차선이 감지가 되나, 중앙을 중심으로 많이 벗어난 경우
                if (abs((160-H2LD)-(H2RD-160)) > 10 and H2LD != 0 and H2RD != 320) or (abs((160-H4LD)-(H4RD-160)) > 30 and H4LD != 0 and H4RD != 320):
                    # 오른쪽으로 많이 기울어 진 경우 왼쪽으로 조금 회전, 왼쪽으로 많이 기울어 진 경우 오른쪽으로 조금 회전
                    if ((160-H2LD)-(H2RD-160) > 0 and H2LD != 0 and H2RD != 320) or ((160-H4LD)-(H4RD-160) > 0 and H4LD != 0 and H4RD != 320): command = 'S1140E'
                    else: command = 'S1160E'
                # 차가 중심에서 크게 벗어나지 않은 경우 직진
                else:
                    command = 'S1150E'
            else:
                # 감지되지 않는 오른쪽 차선의 개수를 센다
                temp1 = 0
                for i in ra:
                    if i == 320: temp1 += 1

                # 감지되지 않는 왼쪽 차선의 개수를 센다
                temp2 = 0
                for i in la:
                    if i == 0: temp2 += 1
                
                # 오른쪽 차선이 더 적게 감지가 된 경우 오른쪽으로 회전, 왼쪽 차선이 더 적게 감지가 된 경우 왼쪽으로 회전
                if temp1 > temp2: command = 'S1190E'
                elif temp1 < temp2: command = 'S1110E'
            
    return command, status, light
 
 
 
 
 
 

