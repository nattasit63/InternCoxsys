from tkinter import filedialog
from tkinter.filedialog import asksaveasfile
from tkinter import *
class Write_file():
    def __init__(self):
        self.customer_amount = 0
        self.depot_amount = 0 
        self.entry_data = []
        self.depot_pos = []
        self.customer_pos = []
        self.all_point_pos = []
        self.max_vehicle_depot = 0
        self.max_load_vehicle = 0
        self.route_duaration = 0
        self.n = 0
    
    def write(self,data,amount_customer,amount_depot,pos):
        self.customer_amount = amount_customer
        self.depot_amount = amount_depot
        self.entry_data = data
        self.depot_pos = pos[0]
        self.customer_pos = pos[1]
        self.max_vehicle_depot = self.entry_data[0]
        self.max_load_vehicle = self.entry_data[1]
        self.route_duration = self.entry_data[2]

        total_line =  1 + self.depot_amount + self.customer_amount  + self.depot_amount
        total_id = self.depot_amount + self.customer_amount

        root = Tk()
        root.withdraw()
        file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
        root.destroy()
        #write first line
        file.write(str(self.max_vehicle_depot)+' '+str(self.customer_amount)+' '+str(self.depot_amount)+'\n')

        #write second part
        for i in range(0,self.depot_amount):
            file.write(str(self.route_duration)+' '+str(self.max_load_vehicle)+'\n')

        #write customer (third part)
        for i in range(1,self.customer_amount+1):
            file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+str(data[i+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')

        #write depot (fourth part)
        for i in range(1,self.depot_amount+1):
            file.write(str(i+self.customer_amount)+' '+str(pos[0][i-1][0])+' '+str(pos[0][i-1][1])+' '+'0'+'   '+'0'+' '+'0'+' '+'0'+'\n')

        file.close()
        print('Configuration File has been write !')
        return file.name
    

    def write2(self,data,all_vp,index_customer_in_all_vp,amount_depot,pos,index_connect):
        self.all_point_amount = all_vp
        self.depot_amount = amount_depot  
        self.entry_data = data
        self.depot_pos = pos[0]
        self.all_point_pos = pos[1]
        self.max_vehicle_depot = self.entry_data[0]
        self.max_load_vehicle = self.entry_data[1]
        self.route_duration = self.entry_data[2]

        print('Entry Data : ',self.entry_data)
        # total_line =  1 + self.depot_amount + self.all_point_pos  + self.depot_amount
        # total_id = self.depot_amount + self.all_point_pos
        
        root = Tk()
        root.withdraw()
        file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
        root.destroy()
        #write first line
        file.write(str(self.max_vehicle_depot)+' '+str(int(self.all_point_amount)-int(self.depot_amount))+' '+str(self.depot_amount)+'\n')

        #write second part
        for i in range(0,self.depot_amount):
            file.write(str(self.route_duration)+' '+str(self.max_load_vehicle)+'\n')

        #write customer (third part)
        print('customer id : ',index_customer_in_all_vp )
        for i in range(1,self.all_point_amount+1):
            for j in index_customer_in_all_vp:
                if int(i)==int(j):
                    self.n+=1
                    # print('n  =',self.n)
                    file.write(str(int(i)-int(self.depot_amount))+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+str(data[self.n+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')
                    break
                    # file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')
                    # break
            for k in index_connect:
                if int(i)==int(k):
                    file.write(str(int(i)-int(self.depot_amount))+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')
                    break



        # for i in range(1,self.all_point_amount+1):
        #     print('point amount : ',str(self.all_point_amount),i)
        #     for j in range(len(index_customer_in_all_vp)):
        #         print('customer id : ',index_customer_in_all_vp , j)
        #         if int(i)==int(index_customer_in_all_vp[j]):
        #             print('MATCH : ',(i,j))
        #             print(i ,index_customer_in_all_vp[j] )
        #             file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+str(data[i+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')
        #         else:
        #             file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')

        #write depot (fourth part)
        for i in range(1,self.depot_amount+1):
            file.write(str(i+self.all_point_amount-self.depot_amount)+' '+str(pos[0][i-1][0])+' '+str(pos[0][i-1][1])+' '+'0'+'   '+'0'+' '+'0'+' '+'0'+'\n')

        file.close()
        print('Configuration File with edge has been write !')
        return file.name
    
    def write_sol(self,cost,sol):
        root = Tk()
        root.withdraw()
        file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
        root.destroy()
        #write first line
        file.write(str(cost)+'\n')
        file.write(str(sol))
        file.close()
        print('Solution file has been write!')
    
    def get_sol_info(self,sol_path):  
        depot_sep_route =[]
        sol = open(sol_path,'r')  
        lines = sol.readlines()
        total_route_line = len(lines)-1
        # get cost data
        optimize_cost = lines[0]
        # info vehicle
        total_depot = int(lines[-1][0])
        for j in range(0,total_depot):
            depot_sep_route.append([])
        for i in range(1,total_route_line+1):
            test = lines[i]
            this_line = test.split("\t")
            this_line_route = this_line[-1].split('\n')[0].replace(' ',',').split(",")
            depot_id = int(this_line[0])-1
            depot_sep_route[depot_id].append(this_line_route)
        return depot_sep_route

    def get_config_info(self,config_path):
        split_data=[]
        customer_pos_info = []
        depot_pos_info = []
        con = open(config_path,'r')
        lines = con.readlines()
        total_line = len(lines)
        major_line = lines[0].split()
        veh_each_depot = major_line[0]
        total_customer = int(major_line[1])
        total_depot = int(major_line[2])
        skip_line = int(total_depot) +1
        
        for i in range(skip_line,total_line):  #split data
            split_data.append(lines[i].split())

        for j in range(len(split_data)-total_depot): #append customer pos
            customer_pos_info.append([int(split_data[j][1]),int(split_data[j][2])])
        
        for k in range(skip_line+total_customer-total_depot-1,len(split_data)):
            depot_pos_info.append([int(split_data[k][1]),int(split_data[k][2])])

        return depot_pos_info,customer_pos_info
    
    def matcher(self,sol_path,config_path):
        depot_id = 0
        pos_each_route = []
        self.raw_route = self.get_sol_info(sol_path)
        depot_pos_info,customer_pos_info = self.get_config_info(config_path)
        total_depot= len(self.raw_route)

        for j in range(0,total_depot):
            pos_each_route.append([])
        try :
            if len(depot_pos_info)==len(self.raw_route):
                print('Matched !')
                for route in self.raw_route:
                    depot_id+=1
                    # print('--------------------------')
                    for i in range(len(route)):
                        for j in range(len(route[i])):
        
                            if route[i][j]==str(0):
                                route[i][j] = depot_id
                # print(raw_route)    
                depot_id = 0
                for new_route in self.raw_route:
                    depot_id+=1
                    for i in range(len(new_route)):
                        for j in range(len(new_route[i])):
                            # print(new_route[i][j])
                            check_depot = isinstance(new_route[i][j],int)
                            if check_depot:
                                pos_each_route[depot_id-1].append([depot_pos_info[depot_id-1]])
                                new_route[i][j] = depot_pos_info[depot_id-1]
                            else :
                                new_route[i][j] = customer_pos_info[int(new_route[i][j])-1]
                print('ROUTE_POS :')
                return self.raw_route
        except:
            return 'Fail ! your files are not match'