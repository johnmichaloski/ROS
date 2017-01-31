# puma.mpl
# Jacobians for the PUMA robot

with(LinearAlgebra);

# From JJ Craig, p. 80

# R_1_0 is R from 1 to the 0,
# 0
#  R
# 1

R_1_0 := <<cos(th1),sin(th1),0>|<-sin(th1),cos(th1),0>|<0,0,1>>;
R_0_1 := Transpose(R_1_0);
P_1_0 := <0,0,0>;
T_1_0 := <<cos(th1),sin(th1),0,0>|<-sin(th1),cos(th1),0,0>|<0,0,1,0>|<0,0,0,1>>;

R_2_1 := <<cos(th2),0,-sin(th2)>|<-sin(th2),0,-cos(th2)>|<0,1,0>>;
R_1_2 := Transpose(R_2_1);
P_2_1 := <0,0,0>;
T_2_1 := <<cos(th2),0,-sin(th2),0>|<-sin(th2),0,-cos(th2),0>|<0,1,0,0>|<0,0,0,1>>;
# P_2_1 := <a1,0,0>;
# T_2_1 := <<cos(th2),0,-sin(th2),0>|<-sin(th2),0,-cos(th2),0>|<0,1,0,0>|<a1,0,0,1>>;

R_3_2 := <<cos(th3),sin(th3),0>|<-sin(th3),cos(th3),0>|<0,0,1>>;
R_2_3 := Transpose(R_3_2);
P_3_2 := <a2,0,d3>;
T_3_2 := <<cos(th3),sin(th3),0,0>|<-sin(th3),cos(th3),0,0>|<0,0,1,0>|<a2,0,d3,1>>;

R_4_3 := <<cos(th4),0,-sin(th4)>|<-sin(th4),0,-cos(th4)>|<0,1,0>>;
R_3_4 := Transpose(R_4_3);
P_4_3 := <a3,d4,0>;
T_4_3 := <<cos(th4),0,-sin(th4),0>|<-sin(th4),0,-cos(th4),0>|<0,1,0,0>|<a3,d4,0,1>>;

R_5_4 := <<cos(th5),0,sin(th5)>|<-sin(th5),0,cos(th5)>|<0,-1,0>>;
R_4_5 := Transpose(R_5_4);
P_5_4 := <0,0,0>;
T_5_4 := <<cos(th5),0,sin(th5),0>|<-sin(th5),0,cos(th5),0>|<0,-1,0,0>|<0,0,0,1>>;

R_6_5 := <<cos(th6),0,-sin(th6)>|<-sin(th6),0,-cos(th6)>|<0,1,0>>;
R_5_6 := Transpose(R_6_5);
P_6_5 := <0,0,0>;
T_6_5 := <<cos(th6),0,-sin(th6),0>|<-sin(th6),0,-cos(th6),0>|<0,1,0,0>|<0,0,0,1>>;

V_1_1 := <0,0,0>;
W_1_1 := <0,0,dth1>;

V_2_2 := MatrixVectorMultiply(R_1_2, Add(V_1_1, CrossProduct(W_1_1, P_2_1)));
W_2_2 := Add(MatrixVectorMultiply(R_1_2, W_1_1), <0,0,dth2>);

V_3_3 := MatrixVectorMultiply(R_2_3, Add(V_2_2, CrossProduct(W_2_2, P_3_2)));
W_3_3 := Add(MatrixVectorMultiply(R_2_3, W_2_2), <0,0,dth3>);

V_4_4 := MatrixVectorMultiply(R_3_4, Add(V_3_3, CrossProduct(W_3_3, P_4_3)));
W_4_4 := Add(MatrixVectorMultiply(R_3_4, W_3_3), <0,0,dth4>);

V_5_5 := MatrixVectorMultiply(R_4_5, Add(V_4_4, CrossProduct(W_4_4, P_5_4)));
W_5_5 := Add(MatrixVectorMultiply(R_4_5, W_4_4), <0,0,dth5>);

V_6_6 := MatrixVectorMultiply(R_5_6, Add(V_5_5, CrossProduct(W_5_5, P_6_5)));
W_6_6 := Add(MatrixVectorMultiply(R_5_6, W_5_5), <0,0,dth6>);

R_6_0 := MatrixMatrixMultiply(R_1_0, MatrixMatrixMultiply(R_2_1, MatrixMatrixMultiply(R_3_2, MatrixMatrixMultiply(R_4_3, MatrixMatrixMultiply(R_5_4, R_6_5)))));

# T_6_0 := MatrixMatrixMultiply(T_1_0, MatrixMatrixMultiply(T_2_1, MatrixMatrixMultiply(T_3_2, MatrixMatrixMultiply(T_4_3, MatrixMatrixMultiply(T_5_4, T_6_5)))));

# Vaug_6_6 := <V_6_6[1],V_6_6[2],V_6_6[3],1>;
# Waug_6_6 := <W_6_6[1],W_6_6[2],W_6_6[3],1>;

V_6_0 := MatrixVectorMultiply(R_6_0, V_6_6);
W_6_0 := MatrixVectorMultiply(R_6_0, W_6_6);

