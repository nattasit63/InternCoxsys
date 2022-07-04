from select_file import Select
from draw import Draw
import pygame as pg

print('-------------------------------------------------------------------------------')
select = Select()
print('-------------------------------------------------------------------------------')
# draw = Draw()


# # Initial Load Image 
# bg = pg.image.load(select.image_file)    # load image from yaml to overlay on pygame
# bg = pg.transform.scale(bg,(draw.width, draw.height))

#Assign local Variables
state = 0
num_pressadd = 0
added_node =[]
pos_node_click =[]
screen_list =[]

#RUN
while (select.flag_launch==1):
    print('yess')
    # if state==0:
    #     # Initial Load Image 
    #     bg = pg.image.load(select.image_file)    # load image from yaml to overlay on pygame
    #     bg = pg.transform.scale(bg,(draw.width, draw.height))
    #     rect = bg.get_rect()
    #     screen = draw.screen
    #     screen.fill((255,255,255))
    #     rect = rect.move((0,0))

    #     draw.check_close()
    #     screen.blit(bg,rect)
    #     pg.display.set_caption('Add Node , Press Enter to do next step')
    #     state=5

    # elif state==1:
    #     mouse = pg.mouse.get_pos()
    #     pg.display.update()
    #     for event in pg.event.get():
    #         if event.type == pg.QUIT:
    #             pg.quit()
    #             exit()
    #         if event.type == pg.KEYDOWN: #Confrim Node
    #             if event.key == pg.K_RETURN:
    #                 added_node_str = str(added_node)
    #                 added_node_str = added_node_str.replace("[","").replace("]","")
    #                 pg.display.set_caption('Add Edge , Press Enter to do next step')
    #                 state=2                      
                    
    #         elif event.type == pg.MOUSEBUTTONDOWN and event.button==1: #if left click ,Add node at mouse pos
    #             px = mouse[0]
    #             py = mouse[1]
    #             p = [px,py]
    #             pos_node_click.append(p)  #save node click pos
    #             added_node.append(str(num_pressadd))
    #             current_screen = pg.Surface.copy(draw.screen) #draw node with num
    #             screen_list.append(current_screen)
    #             draw.circle(mouse[0],mouse[1],draw.width//70)               
    #             draw.buildtext(num_pressadd,mouse[0]-draw.width//140,mouse[1]-draw.width//70)                  
    #             rmouse=0
    #             num_pressadd+=1    
    #             print(pos_node_click)           
    #         elif event.type == pg.MOUSEBUTTONDOWN and event.button==3:  #if right click ,Undo one time
    #             if num_pressadd>=0 and rmouse==0:  #check num_pressadd for index in screen_list not lower than 0
    #                     del pos_node_click[-1]
    #                     del added_node[-1]
    #                     num_pressadd = num_pressadd-1
    #                     draw.screen.blit(current_screen,(0,0)) 
    #                     rmouse=1  

    
    # elif state==2:
    #     pass

