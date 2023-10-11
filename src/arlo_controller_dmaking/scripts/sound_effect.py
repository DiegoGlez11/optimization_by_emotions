#!/usr/bin/env python3
import rospy, os
# from arlo_controller_dmaking.msg import  play_song_msg
from arlo_controller_dmaking.srv import  play_song, play_songResponse
from os.path import expanduser
from playsound import playsound
from gazebo_msgs.msg import ContactsState
import threading
import pygame
import time

sep = os.sep
home = expanduser("~")
root_sound = f"{home}{sep}catkin_ws{sep}src{sep}arlo_controller_dmaking{sep}sound"


sound_metadata = {}
associated_sound = {}

fragil_list = ["plato", "botella", "mesa", "vidrio", "taza", "mesita"]
forniture_list = ["cama", "buro", "refrigerador", "consola", "puerta", "sofa", "juguetero"]

def have_word(name, word_list ):
    for word in word_list:
        if name.find(word) >= 0:
            return True
    return False


def has_attr(obj, attr):
    try:
        obj[attr]
        return True
    except:
        return False

def name_in_list(collision_name):
    global fragil_list, forniture_list

    if have_word(collision_name, fragil_list):
        associated_sound["fragil"]["funct"]("fragil")     

    if have_word(collision_name, forniture_list):
        associated_sound["forniture"]["funct"]("forniture")

def motor_sound():
    global associated_sound

    associated_sound["motor"]["var2"] = True
    while associated_sound["motor"]["var2"]:
        associated_sound["motor"]["num_chan"].set_volume(0.2)
        associated_sound["motor"]["funct"]("motor")

        while associated_sound["motor"]["num_chan"].get_busy():
            time.sleep(0.5)

def play_motor():
    if not associated_sound["motor"]["num_chan"].get_busy():
        # associated_sound["motor"]["num_chan"].set_volume(0.2)
        m = threading.Thread(target=motor_sound)
        m.start()

def stop_motor():
    associated_sound["motor"]["var2"] = False
    associated_sound["motor"]["num_chan"].stop()


def register_sound(model_name, file_name, num_chan=None, loop=0):
    global sound_metadata, associated_sound

    if not has_attr(associated_sound, model_name):
        associated_sound[model_name] = {"count":0, "name":f"f_{model_name}_1", "model_name": model_name}

    associated_sound[model_name]["file"] = file_name
    associated_sound[model_name]["var"] = False
    # associated_sound[model_name]["num_chan"] = num_chan

    associated_sound[model_name]["loop"] = loop
    if num_chan != None:
        associated_sound[model_name]["num_chan"] = pygame.mixer.Channel(num_chan)
    else:
        associated_sound[model_name]["num_chan"] = None

    # nombre asociado al archivo
    name = model_name
 
    # registro de variable de control
    # sound_metadata[name] = {"var":False, "name": name, "file": file_name}
    
    # if(associated_sound["{name}"]["model_name"] != in_name):
    #     return False
    
    # función para iniciar la reproducción
    funct_ini = f"""
def ini_{name}(in_name,fadeout=0, maxtime=0, sync=True):
    global associated_sound
    
    if in_name.find(associated_sound["{name}"]["model_name"]) < 0:
        return False

    if(associated_sound["{name}"]["var"]):
        return False

    def rep_{name}():
        global associated_sound

        if associated_sound["{name}"]["num_chan"] == None:
            playsound(f"{root_sound}/{file_name}")
            associated_sound["{name}"]["var"] = False
        else:
            #chan = pygame.mixer.Channel(associated_sound["{name}"]["num_chan"])
            chan = associated_sound["{name}"]["num_chan"]
            sound = pygame.mixer.Sound(f"{root_sound}/{file_name}")
            chan.play(sound, loops=associated_sound["{name}"]["loop"], maxtime=maxtime, fade_ms=fadeout )

            if fadeout > 0:
                chan.fadeout(fadeout)
            while chan.get_busy() and sync:
                time.sleep(0.5)
            associated_sound["{name}"]["var"] = False

    associated_sound["{name}"]["var"] = True
    x = threading.Thread(target=rep_{name})
    x.start()
    return True
    """
    exec(funct_ini)

    # registro del hilo de inicio
    reg = f"""associated_sound["{name}"]["funct"] = ini_{name}"""
    exec(reg)
    

# topico que recibe las colisiones del robot
def collisions(msg):
    global associated_sound, associated_sound

    for state in msg.states:
        # atributos del estado
        attrs = dir(state)
        # buscamos los miembros que son collisiones
        for name in attrs:
            if(name.find("collision") >= 0):
                collision_name = getattr(state, name)

                # print(collision_name)
                # sonidos
                associated_sound["crash"]["funct"]("crash");

                associated_sound["chico"]["funct"](collision_name)

                associated_sound["mujer"]["funct"](collision_name)

                associated_sound["caja"]["funct"](collision_name)
                
                name_in_list(collision_name)


# servicio ROS que reproduce un sonido
def play_song_effect(req):
    collision_name = req.name
    # print("-- sound_control_srv",collision_name)

    if has_attr(associated_sound, collision_name):
        
        r1 = associated_sound["sucess"]["funct"](collision_name, sync=False)
        r2 = associated_sound["loss"]["funct"](collision_name, sync=False)
        pygame.mixer.music.set_volume(0.1)
        # if r1 or r2:
        #     pygame.mixer.music.set_volume(0.1)

        #     while associated_sound[collision_name]["var"]:
        #         time.sleep(0.3)
    else:
    
        if collision_name.find("motor_play") >= 0:
            play_motor()

        elif collision_name.find("motor_stop") >= 0:
            stop_motor()

        name_in_list(collision_name)

    if collision_name == "pause_world":
        pygame.mixer.music.pause()
        control_channels("stop")

    if collision_name == "stop_effects":
        control_channels("stop")

    if collision_name == "unpause_world":
        play_music()
        # play_motor()
    
    if collision_name == "unpause_world_min_volume":
        play_music(0.1)
        # play_motor()
        

    if collision_name == "play_world":
        play_music()

    # if collision_name == "stop_world":
    #     pygame.mixer.music.stop()


    return play_songResponse()

def play_music(volume=0.5):
    pygame.mixer.music.unpause()
    if not pygame.mixer.music.get_busy():
        try:
            pygame.mixer.music.play(loops=-1)
        except Exception as e:
            pygame.mixer.music.load(f'{root_sound}/luminance.mp3')
            pygame.mixer.music.play(loops=-1)
    pygame.mixer.music.set_volume(volume)
        

def control_channels(control):
    global associated_sound

    for key in associated_sound:
        chan = associated_sound[key]["num_chan"]

        if control == "pause":
            chan.pause()
        if control == "unpause":
            chan.unpause()
        if control == "stop":
            chan.stop()
            if key == "motor":
                stop_motor()
    


if __name__ == "__main__":

    rospy.init_node("sound_effect_node")
    pygame.mixer.init()
    pygame.mixer.set_num_channels(13)

    register_sound("crash","eval_cond_car_crash.mp3", num_chan=2)
    register_sound("chico", "eval_cond_crie_baby.mp3", num_chan=3, loop=4)
    register_sound("mujer", "eval_cond_scream_female.mp3",num_chan=4)
    register_sound("sucess", "apluse_crowd.mp3",num_chan=5)
    register_sound("loss", "abucheo2.wav", num_chan=6)
    register_sound("motor", "servo5-15per.mp3", num_chan=7)
    register_sound("fragil", "breaking_glass.mp3", loop=2, num_chan=8)
    register_sound("caja", "box fall.mp3", loop=2, num_chan=9)
    register_sound("forniture", "crash_forniture.mp3", num_chan=10)
    
    # topico de las colisiones
    rospy.Subscriber("/robot_collision", ContactsState, collisions)
    rospy.Service("/play_song", play_song, play_song_effect)
    
    print("Nodo sound_effect iniciado con éxito")
    rospy.spin()
