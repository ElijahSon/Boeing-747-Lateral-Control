%% Design Office in Flight Control Automatic : BOEING 747 LATERAL CONTROL
% Fils Elie BOUNGOUERES 
%
% University of Bordeaux, Master Complex System Engineering path AM2AS 
clear, close all; 
%%

variables_declaration; 

% state vector of the system 
x = [beta 
        r
        p
        phi];

% Question A.1.1&2

% System matrix 
A = [ -.0558    -.9968  .0802   .0415 
        .598    -.115   -.0318      0
        -3.05    .388  -0.4650      0
         0       .0805      1       0]

B = [.00729
    -.475
    .153
    0    ]

C = [0  1   0   0]

D = 0                     % dim(D) = 1 x 1


SYS = ss(A,B,C,D);      % With this line, we built the state representation of the system 

% Question A.1.3 Transfert function
s = tf('s');                % Laplace variable declaration 
SYS_tf = tf(SYS)
SYS_zpk = zpk(SYS)


[NUM, DEN] = ss2tf(A,B,C,D)  % State representation of the system to the transfert function
G_stf = tf(NUM, DEN)           % Numerator and Denominator of the transfert function 

% G_ss = C*inv(s*eye(4) - A)*B % On ne peut pas utliser cette forme de
% calcul sous matlab, alors on se refere aux fonctions comme tf ou zpk 


% To find the roots of DEN 
roots_DEN = roots(DEN); 

% get the zeros, the poles and the gain 
[Zeros, Poles, Gain] = tf2zp(NUM, DEN);

% It about to display the zeros, the poles and the gain 
disp('Zeros :'); 
disp(Zeros);
disp('Poles :');
disp(Poles);
disp('Gain :');
disp(Gain);


% % Built again the transfert function as a factor product
% [NUM_s, DEN_s] = zp2tf(Zeros, Poles, Gain);
% 
% % It about to display the ttransfert function like a factors products 
% sys_factored = tf(NUM, DEN);
% disp('Factors of the transfert function :');
% disp(sys_factored);

% Zeros : 
%   -0.4981 + 0.0000i
%   -0.0119 + 0.4878i
%   -0.0119 - 0.4878i
% 
% Poles :
%   -0.0329 + 0.9467i   [  Dutch Roll   ]
%   -0.0329 - 0.9467i   [     Mode      ]
%   -0.5627 + 0.0000i   [It is the Roll Mode    ] 
%   -0.0073 + 0.0000i   [It is the Spiral Mode  ]
% 
% Gain :
%    -0.4750

% Question A.1.4 
% Pulsation wn associated with dutch roll mode
% Le produit des poles imaginaires est egale au carre de la pulsation 

% wn = sqrt((-0.0329 + 0.9467)*(-0.0329 - 0.9467))
damp(SYS)

% A l aide de la fonction damp, on peut ainsi determniner les
% carateristiques relatives au coefficients d amortissement et aux
% frequences des differents mode de fonctionnement du systeme 

% Question A.1.5
% Coeff6icient d'amortissement associe au mode de roulis hollandais 

% D apres la focntion damp, le coefficient d amortissement relatif au mode
% de roulis hollandais est = 3.48e-2 


% Question A.1.6
% Diagramme de Bode de la fonction de transfert G(s) 
G_s = SYS_tf
figure, bode(G_s), grid on, title('Diagramme de bode de la fonction de transfert G(s) ')

% Question A.1.7
% Valeur du gain statique sur le diagramme de Bode 
% La valeur du gain statique peut etre directement reccuperee en basse
% frequence sur le diagramme de Bode mais elle sera indiquee en dB 

% en BF sur le diagramme de Bode, on est a 180 degre de phase et donc ceci
% indique qu'on a un gain statique negatif. Alors: 
Gain_dB = 23.7; 
Gain_Statique = (-1)*10^(Gain_dB/20)

% Question A.1.8
% Trace de la reponse du systeme... beta = 1
% [Y,T,X,P] = initial(SYS)
[Y,T] = initial(SYS,[1;0;0;0])
figure, plot(Y), xlabel('Temps t en secondes'), ylabel('Y(t)'),title('Evolution du systeme a beta=1 en CI '), grid on; 

%% Determination d'une loi de commande classique 

% Question A.2.1 & A.2.2 
% Utilisons la commande rlocus pour représenter l'évolution de la position des pôles dans le plan complexe

% Valeur maximale du coefficient d'amortissement que l'on peut obtenir avec
% une action proportionnelle
figure, rlocus(-SYS), sgrid; title('Evolution des poles dans le plan complexe')

% Question A.2.3 & A.2.4 
feedback_G_s= feedback(G_s,1);

% Question A.2.5
% SYS_feedback = tf(feedback_rp) 
% Diagramme de Bode de la FTBF 
FTBF_s = feedback_G_s/(1+feedback_G_s)
figure, bode(FTBF_s), grid on, title('Diagramme de Bode de la fonction de transfert en boucle fermee ')

% Question A.2.6 
% Valeur du gain statique sur le Diagramme de Bode de la FTBF 
% VAleur de la phase egale a -360 degres alors on a bien un signe positif
% sur la valeur du gain 
gain_dB_feeback = -5.741
gain_feedback = 10^(gain_dB_feeback/20)

% Question A.2.7 & A.2.8
% Nouveau modee de la dynamique de vol 
tau = 3; 
washout_s = tau*s/(1 + tau*s); 

gouverne_s = 10/(s +10)
Kgyr = gain_feedback; 

figure, rlocus(-G_s*gouverne_s*washout_s), sgrid; title('Evolution 2 des poles dans le plan complexe')
% Le coefficient maximale que l'on peut obtenir est egal a 2.69

% Question A.2.10
% Fonction de transfert en boucle fermee obtenue 
feedback(G_s*gouverne_s*washout_s,Kgyr); 
Ftbf_s = G_s*gouverne_s/(1 + feedback(G_s*gouverne_s*washout_s,Kgyr))

% Question A.2.11
bode(Ftbf_s), grid on; title("Diagramme de Bode de la FTBF ")
bode(feedback(G_s*gouverne_s*washout_s,Kgyr)), grid on; 

% Question A.2.12
% Reponse du systeme a une condition initiale sur le glissement 
step(-G_s*gouverne_s*washout_s), grid on; title('Reponse du systeme à beta=1 en CI ')
[Y1,T1] = initial(washout_s*SYS*gouverne_s,[1;0;0;0;0;0])
figure, plot(T1,Y1), xlabel('Temps t en secondes'), ylabel('Y(t)'),title('Evolution du systeme a beta=1 en CI '), grid on; 

%% Dtermination d'une loi de commande par placement de poles 
%% Question A.3.1
% Les matrices Ae, Be, ce et De sont obtenues en faisant le calcul qui
% suit dans l'ordre decrit dans le schema fonctionnel
syms rho real 

SYS2 = washout_s*SYS*gouverne_s

Ae = [SYS2.A               zeros(6,6)
     -(SYS2.C)'*SYS2.C    -(SYS2.A)'] 


Be = [SYS2.B
        zeros(6,1)   ]

Ce = [zeros(6,1)'     (SYS2.B)']
De = 0

% Expression de la matrice H 
H = Ae - 1/rho*Be*Ce  

rsqsiso(Ae,Be,Ce,De) ; 
%% Question A.3.2
% Realisation d une fonction rsqsiso
%% SECTION HELP FUNCTION
help roots 
help rlocus
help sgrid 
help feedback 
help initial 
