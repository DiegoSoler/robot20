#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []

ids_possiveis_tags = [11,12,13,21,22,23] # Baseado no enunciado do projeto

area = 0.0 # Variavel com a area do maior contorno colorido

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos pela MobileNet

x = 0
y = 0
z = 0 

ofx = 640
ofy = 480

tfl = 0

tf_buffer = tf2_ros.Buffer()

marcadores_vistos = []

# Matriz de projeçao da camera. Pode ser obtida com rostopic echo /camera/rgb/camera_info
# Veja mais sobre a matematica relevante aqui, se precisar
# http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
#P = np.array([[530.4669406576809, 0.0, 320.5, -37.13268584603767],[0.0, 530.4669406576809, 240.5, 0.0],[0.0, 0.0, 1.0, 0.0]], dtype=float)
#P = np.array([[530.4669406576809, 0.0, 0, -37.13268584603767],[0.0, 530.4669406576809, 240.5, 0.0],[0.0, 0.0, 1.0, 0.0]], dtype=float)
#P = np.array([[-530.4669406576809, 0.0, 0, +37.13268584603767],[0.0, -530.4669406576809, -240.5, 0.0],[0.0, 0.0, 1.0, 0.0]], dtype=float)
# P = np.array([[-530.4669406576809, 0.0, 320.5, -37.13268584603767],[0.0, -530.4669406576809, -240.5, 0.0],[0.0, 0.0, 1.0, 0.0]], dtype=float)
P = np.array([[530.4669406576809, 0.0, 320.5, -37.13268584603767],[0.0, 530.4669406576809, 240.5, 0.0],[0.0, 0.0, 1.0, 0.0]], dtype=float)

# Os pontos em coordenadas do creeper chegam atraves dos alvar bundles. Só que lá estão em cm e precisam ser convertidos em m
# https://github.com/arnaldojr/my_simulation/blob/master/alvar_bundles/markers_11_azul.xml  
tag_big = np.array(([[-0.075,-0.075,0.013,1.0],[0.075,-0.075,0.013,1.0 ],[0.075,0.075,0.013,1.0],[-0.075,0.075,0.013, 1.0]]))
tag_small = np.array([[-0.0149,-0.3423,0.058, 1.0],[0.0149,-0.3423,0.058, 1.0],[0.0149,-0.3111,0.058, 1.0],[-0.0149,-0.3111,0.058, 1.]])
tags = [tag_big, tag_small]

def random_color():
    """Retorna uma cor aleatória"""
    h = np.random.randint(low=0,high=180) # It's easier to randomize colors in the H component
    s = 200
    v = 255
    hsv_img = np.array([[[h,s,v]]], dtype=np.dtype('u1')) # We create a one-pixel image in HSV
    #print(hsv_img)
    rgb_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
    color_val = rgb_img[0][0].astype(float)
    #color_val/=255
    #print(color_val)
    return color_val

def project1(M,P,p):
    """Converte um ponto de coordenadas camera_rgb_optical_frame para coordenadas de pixel"""
    pcamera = np.dot(M, p)
    pscreen = np.dot(P, pcamera)
    p_float = np.true_divide(pscreen[:-1], pscreen[-1])
    p_int = np.array(p_float, dtype=np.int32)
    return p_int



def convert_and_draw_tags(M,P, tag_points, image, color, id):
    """Usando a matriz M que converte do marker para camera_rgb_optical_frame
        e a matriz P imutável da câmera
        Desenha o contorno de onde o tag foi detectado
    """
    points = []
    mx = 0
    my = 0
    for p in tag_points:
        # Dá para melhorar este for e este
        # project vetorizando - todos os pontos de 1 vez
        p_screen = project1(M,P,p)
        p_screen[1] = p_screen[1] + ofy
        p_screen[0] = p_screen[0] + ofx
        mx+=p_screen[0]
        my+=p_screen[1]
        points.append(list(p_screen))
    sz = len(tag_points)    
    mx/=sz # daria para melhorar esta conta
    my/=sz # se a media fosse feita antes de converter para int     
    p_array = np.array([points], dtype=np.int32)    
    cv2.polylines(image, [p_array], True, color,3)
    centro_tag = (int(mx), int(my))
    cv2.circle(image, centro_tag,3, color, 3, 8, 0)    
    font = cv2.FONT_HERSHEY_SIMPLEX 
    font_scale = 1
    font_thickness = 2
    cv2.putText(image, str(id), (int(mx)+15, int(my)), font, font_scale, color, font_thickness, cv2.LINE_AA)
    
    

def faz_transformacao(ref1, ref2):
    """Realiza a transformacao do ponto entre o referencial 1 e o referencial 2 
        retorna a trasnformacao

        Para saber todos os referenciais disponíveis veja o frames.pdf gerado por: 

        rosrun tf view_frames 
    """
    possivel_converter = tf_buffer.can_transform(ref1, ref2, rospy.Time(0))
    if possivel_converter:
        transf = tf_buffer.lookup_transform(ref1, ref2, rospy.Time(0))
        return transf
    else:
        return  None

def decompoe(transf):
    """Recebe uma transformacao de sistemas de coordenadas e a converte em x,y,z e ângulo em RAD em relação a z"""
    # Separa as translacoes das rotacoes
    x = transf.transform.translation.x
    y = transf.transform.translation.y
    z = transf.transform.translation.z
    # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
    # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
    # no eixo X do robo (que e'  a direcao para a frente)
    t = transformations.translation_matrix([x, y, z])
    # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
    r = transformations.quaternion_matrix([transf.transform.rotation.x, transf.transform.rotation.y, transf.transform.rotation.z, transf.transform.rotation.w])
    m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
    z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
    v2 = numpy.dot(m, z_marker)
    v2_n = v2[0:-1] # Descartamos a ultima posicao
    n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
    x_robo = [1,0,0]
    cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
    angulo_marcador_robo = math.acos(cosa)
    return x,y,z, angulo_marcador_robo

def insere_coords_dict(dici, x,y,z,alpha):
    " Insere coordenadas (vindas de um transformation) num dicionário para facilitar uso posterior"
    dici["x"] = x 
    dici["y"] = y 
    dici["z"] = z 
    dici["alpha"] = alpha
    dici["graus"] = math.degrees(alpha)

 
def recebe(msg):
    "Recebe o evento das tags alvar"
    global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y
    global z
    global id

    frame_names = {"camera_rgb_optical_frame":"Coordenadas da câmera", "end_effector_link": "Cubo vermelho da mão", "base_link": "Base do robô"}
    frame_coords = {"camera_rgb_optical_frame":dict(), "end_effector_link": dict(), "base_link":dict()}

    markers_on_screen = []

    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)
        markers_on_screen.append(id)

        referenciais = frame_names.keys()
        for ref in referenciais: 
            transf = faz_transformacao(ref, marcador)
            if transf is not None:           
                xt, yt, zt, alpha_t = decompoe(transf)
                insere_coords_dict(frame_coords[ref], xt, yt, zt, alpha_t)

        for ref in frame_names.keys():
            print("\r")
            if ref in frame_coords.keys():
                print("Marcador ", id)
                print("No referencial :",ref, " que é ", end=None)
                print(frame_names[ref])
                for k in frame_coords[ref].keys():
                    print("%s %5.3f"%(k, frame_coords[ref][k]), end=" ")
                print()
    global marcadores_vistos
    marcadores_vistos = markers_on_screen
    desenha_tags_tela(marcadores_vistos, cv_image)


def desenha_tags_tela(markers_vistos, imagem_bgr):  
        print("DESENHA ", markers_vistos)

        camera = "camera_rgb_optical_frame" # Se usar a camera da garra troque este nome       
        for id in markers_vistos: 
            if id in ids_possiveis_tags: # double check para evitar tags fantasmas
                marcador = "ar_marker_" + str(id)            
                #transf = faz_transformacao(camera, marcador)
                transf = faz_transformacao(marcador, camera)
                if transf is not None:
                    # Monta a matriz de traslacao - isso eh trabalhoso no OS  
                    t = transformations.translation_matrix([transf.transform.translation.x, transf.transform.translation.y, transf.transform.translation.z])
                    # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
                    r = transformations.quaternion_matrix([transf.transform.rotation.x, transf.transform.rotation.y, transf.transform.rotation.z, transf.transform.rotation.w])
                    m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes que converte do frame do marcador para o frame da camera
                    for t in tags:
                        convert_and_draw_tags(m,P, t, imagem_bgr, random_color(), id)

def projeta(m, coords):
    "Recebe uma matriz translacao/rotacao M e  lista de coordenadas homogeneas (com 4 elementos), e os projeta na tela"
    return numpy.dot(m, coords)
    # saida = []
    # for coord in coords:
    #     proj = numpy.dot(m, coords)
    #     saida.append(proj)



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    # print("frame")
    global cv_image
    global media
    global centro
    global resultados

    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass
        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = np.zeros((480*3, 640*3,3 ), dtype=np.uint8)
        cv_image[ofy:saida_net.shape[0]+ofy,ofx:saida_net.shape[1]+ofx,:] =saida_net 
        saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":

    print("""

ALERTA:







Para funcionar este programa *precisa* do Rviz rodando antes:

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

roslaunch my_simulation rviz.launch

"""
    
    
    )

    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados MobileNEt
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        sentido = 1
        while not rospy.is_shutdown():
            for r in resultados:                
                pass # Para evitar prints
            #   print(r)
            vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-math.pi/360))
            velocidade_saida.publish(vel)
            #sentido*=-1

            # marcadores_vistos = []

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                # desenha_tags_tela(marcadores_vistos, cv_image)
                cv2.imshow("cv_image no loop principal", cv_image)
                cv2.waitKey(1)
            rospy.sleep(0.05)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


