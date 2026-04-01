%% Análisis de un robot manipulador tipo SCARA
% Primer Parcial. Cruz Ramírez Dafne Aislinn. Robótica 2026-2.
%% *Índice*
% 1. Introducción
% 
% 2. Modelado cinemático de la postura del robot  
% 
% 2.1 Definición de funciones  
% 
% 2.2 Modelado del robot SCARA  
% 
% 2.3 Planteamiento del modelo cinemático de la posición  
% 
% 2.3.1 Cinemática directa de la posición  
% 
% 2.3.2 Cinemática inversa de la posición  
% 
% 3. Modelado cinemático de las velocidades  
% 
% 3.1 Modelo cinemático directo de las velocidades  
% 
% 3.1.1 Velocidades lineales  
% 
% 3.1.2 Velocidades angulares  
% 
% 3.2 Modelo cinemático inverso de las velocidades  
% 
% 4. Modelado cinemático de las aceleraciones  
% 
% 4.1 Cinemática directa de las aceleraciones  
% 
% 4.1.1 Aceleraciones lineales  
% 
% 4.1.2 Aceleración angular  
% 
% 4.2 Cinemática inversa de las aceleraciones  
% 
% 5. Modelado dinámico  
% 
% 5.1 Modelo dinámico por ecuaciones de Euler-Lagrange  
% 
% 5.1.1 Cálculo de la posición de los centros de masa  
% 
% 5.1.2 Cálculo de las velocidades  
% 
% 5.1.3 Cálculo de las velocidades angulares  
% 
% 5.1.4 Definición de los elementos de inercia  
% 
% 5.1.5 Cálculo del Lagrangiano  
% 
% 5.1.6 Cálculo de los pares  
% 
% 5.1.7 Cálculo de la matriz de inercia  
% 
% 5.1.8 Cálculo del vector de pares  
% 
% 5.9 Cálculo del modelo dinámico inverso  
% 
% 6. Conclusiones  
%% 1. Introducción
% En el presente trabajo se desarrolla el análisis de un robot manipulador tipo 
% SCARA, con el objetivo de aplicar los conceptos de cinemática y dinámica vistos 
% en clase. Este tipo de robot es ampliamente utilizado en la industria debido 
% a su alta precisión y rapidez en tareas de ensamblaje y manipulación de materiales, 
% principalmente en movimientos dentro de un plano.
% 
% El estudio de este manipulador permite comprender la relación existente entre 
% las variables articulares y el movimiento del efector final. A través del modelo 
% cinemático directo, es posible determinar la posición y orientación del robot 
% a partir de los ángulos de sus articulaciones, mientras que el modelo inverso 
% permite calcular dichos ángulos a partir de una posición deseada.
% 
% Asimismo, se analizan las velocidades y aceleraciones del sistema, lo cual 
% resulta fundamental para describir el comportamiento dinámico del robot. Este 
% análisis se complementa con el desarrollo del modelo dinámico mediante el enfoque 
% de Euler-Lagrange, el cual permite determinar los torques necesarios para generar 
% el movimiento del manipulador.
% 
% El desarrollo de estos modelos no solo tiene un enfoque teórico, sino también 
% práctico, ya que se implementan mediante herramientas computacionales como MATLAB, 
% facilitando la simulación y validación del comportamiento del robot. De esta 
% manera, se logra una comprensión integral del funcionamiento de sistemas robóticos 
% utilizados en la automatización industrial. 
% 
% Este tipo de análisis es fundamental en aplicaciones de control y diseño de 
% manipuladores industriales, donde se requiere precisión y eficiencia en el movimiento.
%% 2. Modelado cinemático de la postura del robot
% 2.1 Definición de funciones
% En esta sección se define la función de transformación homogénea, la cual 
% es una herramienta matemática fundamental en robótica para describir la posición 
% y orientación entre diferentes sistemas de referencia.
% 
% Primero, se establece una función general que permite representar tanto rotaciones 
% como traslaciones en el espacio tridimensional. Esta función recibe como parámetros 
% las coordenadas de desplazamiento en los ejes (x), (y) y (z), así como los ángulos 
% de rotación correspondientes.
% 
% Posteriormente, esta función será utilizada para construir las matrices de 
% transformación de cada eslabón del robot, permitiendo describir su movimiento 
% de manera sistemática desde la base hasta el efector final.

%Definición de la función de manera simbolica
syms Tij(x_i_j,y_i_j,z_i_j,gi_j,bi_j,ai_j)

%Definición de la transformación homógenea general
Tij(x_i_j,y_i_j,z_i_j,gi_j,bi_j,ai_j) = [cos(ai_j)*cos(bi_j) cos(ai_j)*sin(bi_j)*sin(gi_j)-sin(ai_j)*cos(gi_j) sin(ai_j)*sin(gi_j)+cos(ai_j)*sin(bi_j)*cos(gi_j) x_i_j; sin(ai_j)*cos(bi_j) cos(ai_j)*cos(gi_j)+sin(ai_j)*sin(bi_j)*sin(gi_j) sin(ai_j)*sin(bi_j)*cos(gi_j)-cos(ai_j)*sin(gi_j) y_i_j; -sin(bi_j) cos(bi_j)*sin(gi_j) cos(bi_j)*cos(gi_j) z_i_j; 0 0 0 1]
% 2.2 Modelado del robot Scara
% A continuación, se muestra la configuración del robot tipo SCARA, en donde 
% se identifican los eslabones y las variables articulares utilizadas en el modelo.
% 
% 
% 
% 
% 
% En la figura se observa la estructura del robot manipulador tipo SCARA, el 
% cual está compuesto por tres eslabones y tres articulaciones rotacionales. Las 
% variables articulares θ1, θ2 y θ3 representan los ángulos de cada unión, mientras 
% que L1, L2 y L3 corresponden a las longitudes de los eslabones. Este modelo 
% permite describir el movimiento del efector final en el plano, siendo la base 
% para el desarrollo del modelo cinemático y dinámico del sistema.
% 
% Para una mejor comprensión del modelo, se presenta a continuación la representación 
% gráfica del robot manipulador tipo SCARA.

% Representación gráfica de la configuración del robot SCARA
figure
hold on
axis equal

% Eslabones
L1 = 1; L2 = 0.8; L3 = 0.5;

theta1 = pi/4;
theta2 = pi/6;
theta3 = pi/3;

% Puntos
x0 = 0; y0 = 0;
x1 = L1*cos(theta1); y1 = L1*sin(theta1);
x2 = x1 + L2*cos(theta1+theta2); 
y2 = y1 + L2*sin(theta1+theta2);
x3 = x2 + L3*cos(theta1+theta2+theta3);
y3 = y2 + L3*sin(theta1+theta2+theta3);

% Dibujar
plot([x0 x1],[y0 y1],'-o')
plot([x1 x2],[y1 y2],'-o')
plot([x2 x3],[y2 y3],'-o')

title('Robot SCARA')
xlabel('X')
ylabel('Y')
grid on
% 2.3 Planteamiento del modelo cinemático de la posición
% Para el desarrollo del modelo cinemático, se definen sistemas de referencia 
% en cada uno de los eslabones del robot. A partir de estos sistemas, se construyen 
% las matrices de transformación homogénea que permiten describir el movimiento 
% del manipulador desde la base hasta el efector final.
% 2.3.1 Cinemática directa de la posición
% La cinemática directa consiste en determinar la posición y orientación del 
% efector final a partir de los valores de las variables articulares del robot.
% 
% Para ello, primero se definen las matrices de transformación homogénea entre 
% cada par de eslabones, considerando las rotaciones generadas por los ángulos 
% articulares y las longitudes de los eslabones.
% 
% Posteriormente, estas matrices se multiplican en orden desde la base hasta 
% el efector final, obteniendo así una matriz total que describe completamente 
% la postura del robot en el espacio.

syms x_O_1 y_O_1 theta_O_1 L_2 theta_1_2 L_3 theta_2_3 L_1

T_O_1 = Tij(x_O_1,y_O_1,0,0,0,theta_O_1)
T_1_2 = Tij(L_1,0,0,0,0,theta_1_2)
T_2_3 = Tij(L_2,0,0,0,0,theta_2_3)
T_3_P = Tij(L_3,0,0,0,0,0)

T_O_P = simplify(T_O_1*T_1_2*T_2_3*T_3_P)
%% 
% La matriz de transformación homogénea ( T_{0}^{P} ) representa la posición 
% y orientación del efector final respecto al sistema de referencia base. A partir 
% de esta matriz, es posible extraer el vector de posición y la orientación del 
% robot.
% 
% Las coordenadas del efector final se obtienen de la última columna de la matriz, 
% mientras que la orientación está relacionada con la suma de los ángulos articulares.
% 
% El vector de postura del robot representa la posición y orientación del efector 
% final respecto al sistema de referencia base. Este vector está compuesto por 
% las coordenadas en el plano (x) y (y), así como la orientación del robot, la 
% cual corresponde a la suma de los ángulos articulares.
% 
% Dicho vector se obtiene directamente a partir de la matriz de transformación 
% homogénea, tomando los elementos de la última columna para la posición y sumando 
% los ángulos para la orientación.

xi_O_P = [T_O_P(1,4);T_O_P(2,4);theta_O_1+theta_1_2+theta_2_3]
% 2.3.2 Cinemática Inversa de la posición
% Una vez obtenido el vector de postura del robot, se procede a calcular el 
% Jacobiano del sistema. Este se obtiene derivando dicho vector respecto a cada 
% una de las variables articulares.
% 
% El Jacobiano es una herramienta fundamental, ya que permite relacionar los 
% cambios en las articulaciones con los cambios en la posición y orientación del 
% efector final, lo cual será utilizado posteriormente en el análisis de velocidades 
% y aceleraciones.

q = [theta_O_1; theta_1_2; theta_2_3];

J = simplify(jacobian(xi_O_P,q))
%% 
% El Jacobiano permite identificar configuraciones singulares del robot, las 
% cuales ocurren cuando su determinante es cero. En estas configuraciones, el 
% manipulador pierde grados de libertad y no puede moverse en ciertas direcciones.
%% 3. Modelado cinemático de las velocidades.
% 3.1 Modelo cinemático directo de las velocidades
% Para el análisis de velocidades, se parte del Jacobiano previamente obtenido. 
% Primero se definen las velocidades articulares como variables independientes.
% 
% Posteriormente, se multiplican estas velocidades por el Jacobiano, obteniendo 
% así la velocidad del efector final. Este procedimiento permite entender cómo 
% el movimiento de cada articulación contribuye al movimiento total del robot.
% 
% Se utilizan dos notaciones para las velocidades articulares: dtheta representa 
% derivadas en el contexto cinemático, mientras que theta_dot se emplea en el 
% análisis dinámico. Ambas representan la misma magnitud física.

syms J_theta

J_theta 
J_theta = jacobian(xi_O_P,[theta_O_1, theta_1_2,theta_2_3])
% 3.1.1 Velocidades Lineales
% Las velocidades lineales del efector final se obtienen multiplicando el Jacobiano 
% por el vector de velocidades articulares, lo cual describe cómo el movimiento 
% de cada articulación contribuye al movimiento del robot.

syms dtheta_O_1 dtheta_1_2 dtheta_2_3

q_dot = [dtheta_O_1; dtheta_1_2; dtheta_2_3];

xi_dot = J_theta*q_dot
% 3.1.2 Velocidades Angulares
% La velocidad angular del efector final corresponde a la suma de las velocidades 
% articulares, debido a la naturaleza serial del manipulador.

omega = dtheta_O_1 + dtheta_1_2 + dtheta_2_3
% 3.2 Modelo cinemático inverso de las velocidades
% El modelo cinemático inverso de las velocidades permite obtener las velocidades 
% articulares a partir de las velocidades del efector final mediante la inversión 
% del Jacobiano.

inv(J_theta)
q_dot_inv = simplify(inv(J_theta)*xi_dot)
%% 
% La inversión del Jacobiano solo es posible cuando este es no singular. En 
% configuraciones singulares, se requieren métodos alternativos como la pseudoinversa.
%% 4. Modelado cinemático de las aceleraciones. 
% 4.1 Cinemática directa de las aceleraciones
% Para obtener las aceleraciones del efector final, se deriva la expresión de 
% velocidades respecto al tiempo. Este proceso no solo considera la aceleración 
% de las articulaciones, sino también el cambio del Jacobiano debido al movimiento 
% del robot.
% 
% Por esta razón, la aceleración se compone de dos términos: uno asociado a 
% las aceleraciones articulares y otro relacionado con la variación del Jacobiano. 
% Esto permite describir de manera más completa el comportamiento dinámico del 
% sistema.

% Derivada del Jacobiano
J_dot = diff(J_theta,theta_O_1)*dtheta_O_1 + ...
        diff(J_theta,theta_1_2)*dtheta_1_2 + ...
        diff(J_theta,theta_2_3)*dtheta_2_3;

syms ddtheta_O_1 ddtheta_1_2 ddtheta_2_3

q_ddot = [ddtheta_O_1; ddtheta_1_2; ddtheta_2_3];

% Aceleración del efector final
xi_ddot = J_theta*q_ddot + J_dot*q_dot
% 4.1.1 Aceleración Lineales
% Las aceleraciones lineales del efector final corresponden a las componentes 
% en los ejes (x) y (y), las cuales se obtienen a partir del vector de aceleración 
% previamente calculado. Estas describen cómo varía la velocidad del efector final 
% en el plano de movimiento.

% Aceleraciones lineales
ax = xi_ddot(1);
ay = xi_ddot(2);

aceleracion_lineal = [ax; ay]
% 4.1.2 Aceleración Angular
% La aceleración angular del efector final está dada por la suma de las aceleraciones 
% articulares, debido a la estructura serial del manipulador.

alpha = ddtheta_O_1 + ddtheta_1_2 + ddtheta_2_3
% 4.2 Cinemática inversa de las aceleraciones
% El modelo cinemático inverso de las aceleraciones permite obtener las aceleraciones 
% articulares a partir de la aceleración del efector final, considerando la influencia 
% de la variación del Jacobiano en el sistema.

q_ddot = inv(J_theta)*(xi_ddot - J_dot*q_dot)
%% 
% 
%% 5. Modelado dinámico
% En esta sección se desarrolla el modelo dinámico del robot, el cual permite 
% relacionar el movimiento del sistema con las fuerzas o torques necesarios para 
% producirlo.
% 
% Para ello, se utiliza el método de Euler-Lagrange, el cual se basa en el análisis 
% de la energía del sistema. Primero se calcula la energía cinética asociada al 
% movimiento de los eslabones, así como la energía potencial debida a la gravedad.
% 
% Posteriormente, a partir del Lagrangiano (definido como la diferencia entre 
% energía cinética y potencial), se obtienen las ecuaciones que describen el comportamiento 
% dinámico del robot.
% 5.1 Modelo dinámico por ecuaciones de Eüler-Lagrange
% El método de Euler-Lagrange se basa en la diferencia entre la energía cinética 
% y potencial del sistema, conocida como Lagrangiano. A partir de este, se obtienen 
% las ecuaciones de movimiento del robot.
% 
% Se emplearan las siguientes ecuaciones para el cálculo de la energía cinética:
% 
% $$\$$
% 
% 
% 
% $$k_{i} = \frac{m_{i}}{2} \mathbf{v}_{C_{i}}^T \mathbf{v}_{C_{i}} + \frac{1}{2} 
% \mathbf{\omega}_{C_{i}}^T \mathbf{I}_{C_{i}} \mathbf{\omega}_{C_{i}}$$
% 
% 

syms x_1_C1 theta_dot_O_1

v_C1_C1 = [0;x_1_C1*theta_dot_O_1;0]

v_O_C1 = [-x_1_C1*sin(theta_O_1)*theta_dot_O_1;x_1_C1*cos(theta_O_1)*theta_dot_O_1;0]
transpose(v_C1_C1)*v_C1_C1
simplify(transpose(v_O_C1)*v_O_C1)


% 5.1.1 Cálculo de la posición de los centros de masa
% Para el análisis dinámico, es necesario determinar la posición de los centros 
% de masa de cada eslabón, ya que estos influyen en el cálculo de la energía cinética 
% y potencial del sistema.

syms x_1_C1 x_2_C2 x_3_C3

T_1_C1 = Tij(x_1_C1,0,0,0,0,0)
T_O_C1 = T_O_1*T_1_C1

T_2_C2 = Tij(x_2_C2,0,0,0,0,0)
T_O_C2 = T_O_1*T_1_2*T_2_C2

T_3_C3 = Tij(x_3_C3,0,0,0,0,0)
T_O_C3 = T_O_1*T_1_2*T_2_3*T_3_C3


%Vectores de posición
p_O_C1 = [T_O_C1(1,4);T_O_C1(2,4);T_O_C1(3,4)]
p_O_C2 = simplify([T_O_C2(1,4);T_O_C2(2,4);T_O_C2(3,4)])

p_O_C3 = simplify([T_O_C3(1,4);T_O_C3(2,4);T_O_C3(3,4)])
% 5.1.2 Cálculo de las velocidades
% Las velocidades lineales de los centros de masa se obtienen derivando sus 
% posiciones respecto al tiempo, lo cual permite calcular la energía cinética 
% del sistema.

syms theta_dot_O_1 theta_dot_1_2 theta_dot_2_3

v_O_C1 = diff(p_O_C1,theta_O_1)*theta_dot_O_1+diff(p_O_C1,theta_1_2)*theta_dot_1_2+diff(p_O_C1,theta_2_3)*theta_dot_2_3
v_O_C2 = diff(p_O_C2,theta_O_1)*theta_dot_O_1+diff(p_O_C2,theta_1_2)*theta_dot_1_2+diff(p_O_C2,theta_2_3)*theta_dot_2_3
v_O_C3 = diff(p_O_C3,theta_O_1)*theta_dot_O_1+diff(p_O_C3,theta_1_2)*theta_dot_1_2+diff(p_O_C3,theta_2_3)*theta_dot_2_3
% 5.1.3 Cálculo de las velocidades angulares
% Las velocidades angulares de cada eslabón están relacionadas con las velocidades 
% articulares del robot, siendo fundamentales para el cálculo de la energía cinética 
% rotacional.

syms omega_1_1 omega_2_2 omega_3_3
%Propagación para el primer cuerpo
omega_1_1
omega_O_O = [0;0;0]
n_1_1 = [0;0;1]
R_O_1 = [T_O_1(1,1),T_O_1(1,2),T_O_1(1,3);T_O_1(2,1),T_O_1(2,2),T_O_1(2,3);T_O_1(3,1),T_O_1(3,2),T_O_1(3,3)]
R_1_O = transpose(R_O_1)

%Ecuación de propagación
omega_1_1 = R_1_O*omega_O_O+n_1_1*theta_dot_O_1

%Propagación para el segundo cuerpo
omega_2_2
n_2_2 = [0;0;1]
R_1_2 = [T_1_2(1,1),T_1_2(1,2),T_1_2(1,3);T_1_2(2,1),T_1_2(2,2),T_1_2(2,3);T_1_2(3,1),T_1_2(3,2),T_1_2(3,3)]
R_2_1 = transpose(R_1_2)

%Ecuación de propagación
omega_2_2 = R_2_1*omega_1_1+n_2_2*theta_dot_1_2

%Propagación para el tercer cuerpo
omega_3_3
n_3_3 = [0;0;1]
R_2_3 = [T_2_3(1,1),T_2_3(1,2),T_2_3(1,3);T_2_3(2,1),T_2_3(2,2),T_2_3(2,3);T_2_3(3,1),T_2_3(3,2),T_2_3(3,3)]
R_3_2 = transpose(R_2_3)

%Ecuación de propagación
omega_3_3 = R_3_2*omega_2_2+n_3_3*theta_dot_2_3
v_O_C3
%% 
% 
% 5.1.4 Definición de los elementos de inercia 
% Se definen los parámetros de masa e inercia de cada eslabón, los cuales son 
% necesarios para el desarrollo del modelo dinámico del robot.

syms g I_xx1 I_yy1 I_zz1 I_xx2 I_yy2 I_zz2 I_xx3 I_yy3 I_zz3
%vector de gravedad

g_v = [0;-g;0]

I_C1 = [I_xx1,0,0;0,I_yy1,0;0,0,I_zz1]
I_C2 = [I_xx2,0,0;0,I_yy2,0;0,0,I_zz2]
I_C3 = [I_xx3,0,0;0,I_yy3,0;0,0,I_zz3]
%% 
% 
% 5.1.5 Cálculo del Lagrangeano
% El Lagrangiano se define como la diferencia entre la energía cinética y la 
% energía potencial del sistema, y constituye la base para obtener las ecuaciones 
% dinámicas del robot.

syms m_1 m_2 m_3
%energía cinética de cada uno de los cuerpos

k_1 = simplify((m_1/2)*transpose(v_O_C1)*v_O_C1+(1/2)*transpose(omega_1_1)*I_C1*omega_1_1)

k_2 = simplify((m_2/2)*transpose(v_O_C2)*v_O_C2+(1/2)*transpose(omega_2_2)*I_C2*omega_2_2)

k_3 = simplify((m_3/2)*transpose(v_O_C3)*v_O_C3+(1/2)*transpose(omega_3_3)*I_C3*omega_3_3)
% Cálculo de la energía potencial de cada cuerpo

u_1 = -m_1*transpose(p_O_C1)*g_v
u_2 = -m_2*transpose(p_O_C2)*g_v
u_3 = -m_3*transpose(p_O_C3)*g_v
%% 
% 


La = (k_1+k_2+k_3)-(u_1+u_2+u_3)
% 5.1.6 Cálculo de los pares

syms theta_ddot_O_1 theta_ddot_1_2 theta_ddot_2_3

D_theta1 = diff(La,theta_dot_O_1)

% Cálculo de relación

tao_1 = diff(D_theta1,theta_O_1)*theta_dot_O_1 + diff(D_theta1,theta_1_2)*theta_dot_1_2 + diff(D_theta1,theta_2_3)*theta_dot_2_3 + diff(D_theta1,theta_dot_O_1)*theta_ddot_O_1+ diff(D_theta1,theta_dot_1_2)*theta_ddot_1_2+ diff(D_theta1,theta_dot_2_3)*theta_ddot_2_3-diff(La,theta_O_1)

D_theta2 = diff(La,theta_dot_1_2)
tao_2 = diff(D_theta2,theta_O_1)*theta_dot_O_1 + diff(D_theta2,theta_1_2)*theta_dot_1_2 + diff(D_theta2,theta_2_3)*theta_dot_2_3 + diff(D_theta2,theta_dot_O_1)*theta_ddot_O_1+ diff(D_theta2,theta_dot_1_2)*theta_ddot_1_2+ diff(D_theta2,theta_dot_2_3)*theta_ddot_2_3-diff(La,theta_1_2)

D_theta3 = diff(La,theta_dot_2_3)
tao_3 = diff(D_theta3,theta_O_1)*theta_dot_O_1 + diff(D_theta3,theta_1_2)*theta_dot_1_2 + diff(D_theta3,theta_2_3)*theta_dot_2_3 + diff(D_theta3,theta_dot_O_1)*theta_ddot_O_1+ diff(D_theta3,theta_dot_1_2)*theta_ddot_1_2+ diff(D_theta3,theta_dot_2_3)*theta_ddot_2_3-diff(La,theta_2_3)

tao = [tao_1;tao_2;tao_3]
% 5.1.7 Cálculo de la matriz de inercia
% El modelo dinámico puede expresarse en forma matricial, donde se identifican 
% la matriz de inercia, los términos centrífugos y de Coriolis, así como el vector 
% de gravedad.

% Cálculo de la matriz de inercia

M1 = subs(tao,[theta_ddot_O_1,theta_ddot_1_2,theta_ddot_2_3,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,g],[1,0,0,0,0,0,0])
M2 = subs(tao,[theta_ddot_O_1,theta_ddot_1_2,theta_ddot_2_3,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,g],[0,1,0,0,0,0,0])
M3 = subs(tao,[theta_ddot_O_1,theta_ddot_1_2,theta_ddot_2_3,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,g],[0,0,1,0,0,0,0])

M_theta = collect([M1 M2 M3],[m_1,m_2,m_3])
%%
M_theta = simplify(M_theta)
% 5.1.8 Cálculo del vector de pares 
% El vector V_theta representa los términos centrífugos y de Coriolis, los cuales 
% dependen de las velocidades articulares del sistema.

V_theta = subs(tao,[theta_ddot_O_1,theta_ddot_1_2,theta_ddot_2_3,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,g],[0,0,0,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,0])
%% 
% El vector G_theta corresponde a los efectos de la gravedad sobre cada uno 
% de los eslabones del robot.

G_theta = subs(tao,[theta_ddot_O_1,theta_ddot_1_2,theta_ddot_2_3,theta_dot_O_1,theta_dot_1_2,theta_dot_2_3,g],[0,0,0,0,0,0,g])
%% 
% El modelo dinámico del robot puede expresarse en forma matricial como:

tao = M_theta*q_ddot + V_theta + G_theta
%% 
% donde:
% 
% M_theta es la matriz de inercia del sistema,
% 
% V_theta representa los efectos centrífugos y de Coriolis,
% 
% G_theta corresponde al vector de efectos gravitacionales.
% 5.9 Cálculo del modelo dinámico inverso
% Finalmente, el modelo dinámico inverso se obtiene a partir de los torques 
% calculados mediante el método de Euler-Lagrange. Estos torques representan directamente 
% las fuerzas necesarias en cada articulación para generar un movimiento específico 
% del robot.
% 
% Este resultado es de gran importancia, ya que permite determinar qué esfuerzos 
% debe realizar el sistema para seguir una trayectoria deseada.

% Modelo dinámico inverso (torques del sistema)
tao_modelo = simplify(tao)
%% 6. Conclusiones
% En el presente trabajo se desarrollaron los modelos cinemáticos y dinámicos 
% de un robot manipulador tipo SCARA, lo cual permitió comprender de manera integral 
% la relación entre las variables articulares y el movimiento del efector final.
% 
% A través del modelado cinemático directo e inverso, se logró analizar cómo 
% se determina la posición del robot a partir de sus articulaciones, así como 
% el proceso inverso para alcanzar una posición deseada. Asimismo, el uso del 
% Jacobiano permitió establecer la relación entre velocidades y aceleraciones, 
% facilitando el análisis del comportamiento dinámico del sistema.
% 
% Por otro lado, mediante el enfoque de Euler-Lagrange, se obtuvieron las ecuaciones 
% que describen el modelo dinámico del robot, permitiendo identificar los torques 
% necesarios para generar el movimiento. Este proceso permitió comprender la importancia 
% de considerar tanto la energía cinética como la potencial en el análisis de 
% sistemas mecatrónicos.
% 
% El uso de MATLAB resultó fundamental para el desarrollo del modelo, ya que 
% permitió trabajar de manera simbólica y validar los resultados obtenidos. Este 
% trabajo permitió reforzar los conocimientos adquiridos en clase y comprender 
% su aplicación en sistemas reales.
% 
% Finalmente, se puede concluir que el modelado matemático es una herramienta 
% esencial en el diseño y control de robots, ya que permite predecir su comportamiento 
% y optimizar su desempeño en aplicaciones industriales.
% 
%