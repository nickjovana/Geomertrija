import math as m
import numpy as np

def normalize(v):
     norm = np.linalg.norm(v)
     if norm == 0: 
         return v
     return v / norm
     

def Euler2A(fi,teta, ksi):
     x=np.array([[1,0,0],[0,m.cos(fi),-m.sin(fi)],[0,m.sin(fi),m.cos(fi)]])
     y=np.array([[m.cos(teta),0,m.sin(teta)],[0,1,0],[-m.sin(teta),0,m.cos(teta)]])
     z=np.array([[m.cos(ksi),-m.sin(ksi),0],[m.sin(ksi),m.cos(ksi),0],[0,0,1]])
     print(z.dot(y).dot(x))
def A2AxisAngle(matrica):
	provera = np.matmul(matrica, np.transpose(matrica))
	print("matica*transponovana:{}".format(provera))
	det=np.linalg.det(matrica)
        print("Determinanta:{}".format(det))
	A=matrica-np.eye(3)
	u1=A[0,:]
	u2=A[1,:]     
	p=np.cross(u1,u2)
	p=normalize(p)
	
	uprim=u2.dot(matrica)
	ugao=m.acos(uprim.dot(u2)/(np.linalg.norm(uprim)*np.linalg.norm(u2)))
	mesoviti=np.array([u1,uprim,p])
	det1 = np.linalg.det(mesoviti)
	
	if det1 < 0:
		p = -p
	print(p)
	print(ugao)

     
     
def Rodrigez(x,y,z,ugao):
     pt=np.array([[x],[y],[z]])
     p=np.transpose(pt)
     px=np.array([[0,-p[0][2],p[0][1]],[p[0][2],0,-p[0][0]],[-p[0][1],p[0][0],0]])
     
     resenje=pt.dot(p)-m.cos(m.pi/2)*(np.eye(3)-pt.dot(p))+m.sin(m.pi/2)*px
     print(resenje)

def A2Euler(matrica):
     a31 = matrica[2][0]
     if  a31<1 :
         if a31>-1:
             x = m.atan2(matrica[1][0] , matrica[0][0])
             y = m.asin(-a31)
             z = m.atan2(matrica[2][1], matrica[2][2])
         else:
             x  = m.atan2(-matrica[0][1], matrica[1][1])
             y = m.pi/2
             z = 0
     else:
         x = m.atan2(-matrica[0][1], matrica[1][1])
         y = -m.pi/2
         z = 0
     print(np.array([z, y, x]))
    
     
def AxisAngle2Q(vektor,ugao):
     w=m.cos(ugao/2)
     p=normalize(vektor)
     p=m.sin(ugao/2)*p
     print(p,w)
     
def Q2AxisAngle(vektor):
     q=normalize(vektor)
     if q[3]<0:
         q=-q
     w=2*m.acos(q[3])
     if q[3]==1 or q[3]==-1:
         p=np.array([1,0,0])
     else:
         p=normalize(np.array([q[0],q[1],q[2]]))
     print(p,w)
def Slerp(q1,q2,tm,t):
	if(t==0):
		return q1
	if(t==tm):
		return q2
	cos=q1.dot(q2)/(np.linalg.norm(q1)*np.linalg.norm(q2))
	if(cos<0):
		q1=-q1
		cos=-cos
	if(cos>0.95):
		return q1
	fi=m.acos(cos)
	return (m.sin(f*(1-t/tm))/m.sin(f))*q1+(m.sin(f*t/tm)/m.sin(f))*q2

print("Euler2Matrix:")     
Euler2A(-m.atan(1/4),-m.asin(8/9),m.atan(4))
print("Matrix2AxisAngle:")
A2AxisAngle(np.array([[0.11111111,-0.88888889,-0.44444444],[0.44444444,0.44444444,-0.77777778],[0.88888889,-0.11111111,0.44444444]]))
print("Rodrigez:")
Rodrigez(0.33333333,-0.666666666,0.666666666,1.5707963317948965)  
print("Matrix2Euler:")
A2Euler(np.array([[0.11111111,-0.88888889,-0.44444444],[0.44444444,0.44444444,-0.77777778],[0.88888889,-0.11111111,0.44444444]]))
print("AxisAngle2Quaternion:")
AxisAngle2Q(np.array([1/3,-2/3,2/3]),m.pi/2)
print("Quaternion2AxisAngle:")
Q2AxisAngle(np.array([ 0.23570226,-0.47140452,0.47140452,0.7071067811865476 ]))



