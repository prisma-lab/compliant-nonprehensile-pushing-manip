/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) pushing_reduced_expl_ode_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s2[48] = {8, 5, 0, 8, 16, 24, 32, 40, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s3[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s4[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s5[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s6[95] = {91, 1, 0, 91, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90};

casadi_real casadi_sq(casadi_real x) { return x*x;}

/* pushing_reduced_expl_ode_hess:(i0[8],i1[8x8],i2[8x5],i3[8],i4[5],i5[10])->(o0[13],o1[91]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  a1=arg[0] ? arg[0][2] : 0;
  a2=cos(a1);
  a3=arg[5] ? arg[5][0] : 0;
  a4=arg[0] ? arg[0][6] : 0;
  a5=arg[3] ? arg[3][1] : 0;
  a6=(a4*a5);
  a6=(a3*a6);
  a7=(a2*a6);
  a8=sin(a1);
  a9=arg[5] ? arg[5][1] : 0;
  a10=arg[0] ? arg[0][7] : 0;
  a11=(a10*a5);
  a11=(a9*a11);
  a12=(a8*a11);
  a7=(a7-a12);
  a12=cos(a1);
  a13=arg[3] ? arg[3][0] : 0;
  a10=(a10*a13);
  a10=(a9*a10);
  a14=(a12*a10);
  a7=(a7-a14);
  a14=sin(a1);
  a15=(a4*a13);
  a15=(a3*a15);
  a16=(a14*a15);
  a7=(a7-a16);
  if (res[0]!=0) res[0][2]=a7;
  a7=arg[5] ? arg[5][8] : 0;
  a16=2.;
  a17=(a7/a16);
  a18=arg[5] ? arg[5][2] : 0;
  a19=arg[3] ? arg[3][2] : 0;
  a4=(a4*a19);
  a4=(a18*a4);
  a4=(a17*a4);
  a20=3.1415926535897931e+00;
  a21=arg[0] ? arg[0][3] : 0;
  a20=(a20-a21);
  a22=cos(a20);
  a23=casadi_sq(a22);
  a4=(a4/a23);
  a24=sin(a21);
  a25=cos(a21);
  a26=(a25+a25);
  a27=(a7/a16);
  a28=arg[4] ? arg[4][0] : 0;
  a29=arg[4] ? arg[4][1] : 0;
  a28=(a28-a29);
  a28=(a27*a28);
  a29=casadi_sq(a25);
  a28=(a28/a29);
  a30=(a28/a29);
  a31=arg[3] ? arg[3][5] : 0;
  a32=(a30*a31);
  a33=(a26*a32);
  a34=(a24*a33);
  a34=(a4-a34);
  if (res[0]!=0) res[0][3]=a34;
  if (res[0]!=0) res[0][4]=a0;
  if (res[0]!=0) res[0][5]=a0;
  a34=sin(a1);
  a34=(a34*a3);
  a34=(a34*a5);
  a35=tan(a20);
  a35=(a17*a35);
  a35=(a18*a35);
  a35=(a35*a19);
  a34=(a34-a35);
  a35=cos(a1);
  a35=(a35*a3);
  a35=(a35*a13);
  a34=(a34+a35);
  if (res[0]!=0) res[0][6]=a34;
  a34=cos(a1);
  a34=(a34*a9);
  a34=(a34*a5);
  a7=(a7/a16);
  a16=arg[5] ? arg[5][9] : 0;
  a7=(a7+a16);
  a7=(a18*a7);
  a7=(a7*a19);
  a34=(a34-a7);
  a7=sin(a1);
  a7=(a7*a9);
  a7=(a7*a13);
  a34=(a34-a7);
  if (res[0]!=0) res[0][7]=a34;
  a34=arg[3] ? arg[3][3] : 0;
  a7=(a31/a29);
  a16=(a27*a7);
  a35=(a34-a16);
  if (res[0]!=0) res[0][8]=a35;
  a16=(a16-a34);
  if (res[0]!=0) res[0][9]=a16;
  a16=arg[3] ? arg[3][6] : 0;
  a34=arg[3] ? arg[3][4] : 0;
  a35=arg[5] ? arg[5][4] : 0;
  a34=(a34/a35);
  a16=(a16+a34);
  if (res[0]!=0) res[0][10]=a16;
  a16=arg[3] ? arg[3][7] : 0;
  a34=arg[5] ? arg[5][5] : 0;
  a34=(a31/a34);
  a16=(a16+a34);
  if (res[0]!=0) res[0][11]=a16;
  if (res[0]!=0) res[0][12]=a0;
  a0=arg[1] ? arg[1][2] : 0;
  a16=arg[1] ? arg[1][6] : 0;
  a34=(a5*a16);
  a34=(a3*a34);
  a34=(a2*a34);
  a35=sin(a1);
  a36=(a35*a0);
  a36=(a6*a36);
  a34=(a34-a36);
  a36=cos(a1);
  a37=(a36*a0);
  a37=(a11*a37);
  a38=arg[1] ? arg[1][7] : 0;
  a39=(a5*a38);
  a39=(a9*a39);
  a39=(a8*a39);
  a37=(a37+a39);
  a34=(a34-a37);
  a37=(a13*a38);
  a37=(a9*a37);
  a37=(a12*a37);
  a39=sin(a1);
  a40=(a39*a0);
  a40=(a10*a40);
  a37=(a37-a40);
  a34=(a34-a37);
  a37=cos(a1);
  a40=(a37*a0);
  a40=(a15*a40);
  a41=(a13*a16);
  a41=(a3*a41);
  a41=(a14*a41);
  a40=(a40+a41);
  a34=(a34-a40);
  a40=(a0*a34);
  a41=arg[1] ? arg[1][3] : 0;
  a42=(a19*a16);
  a42=(a18*a42);
  a42=(a17*a42);
  a42=(a42/a23);
  a4=(a4/a23);
  a22=(a22+a22);
  a43=sin(a20);
  a44=(a43*a41);
  a44=(a22*a44);
  a44=(a4*a44);
  a42=(a42-a44);
  a44=cos(a21);
  a45=(a44*a41);
  a45=(a33*a45);
  a28=(a28/a29);
  a25=(a25+a25);
  a21=sin(a21);
  a46=(a21*a41);
  a47=(a25*a46);
  a48=(a28*a47);
  a48=(a48/a29);
  a30=(a30/a29);
  a49=(a30*a47);
  a48=(a48+a49);
  a48=(a31*a48);
  a48=(a26*a48);
  a46=(a46+a46);
  a46=(a32*a46);
  a48=(a48-a46);
  a48=(a24*a48);
  a45=(a45+a48);
  a42=(a42-a45);
  a45=(a41*a42);
  a40=(a40+a45);
  a45=cos(a1);
  a48=(a45*a0);
  a48=(a3*a48);
  a48=(a5*a48);
  a20=cos(a20);
  a20=casadi_sq(a20);
  a41=(a41/a20);
  a41=(a17*a41);
  a41=(a18*a41);
  a41=(a19*a41);
  a48=(a48+a41);
  a41=sin(a1);
  a46=(a41*a0);
  a46=(a3*a46);
  a46=(a13*a46);
  a48=(a48-a46);
  a16=(a16*a48);
  a40=(a40+a16);
  a16=sin(a1);
  a46=(a16*a0);
  a46=(a9*a46);
  a46=(a5*a46);
  a1=cos(a1);
  a0=(a1*a0);
  a0=(a9*a0);
  a0=(a13*a0);
  a46=(a46+a0);
  a38=(a38*a46);
  a40=(a40-a38);
  if (res[1]!=0) res[1][0]=a40;
  a40=arg[1] ? arg[1][10] : 0;
  a38=(a40*a34);
  a0=arg[1] ? arg[1][11] : 0;
  a49=(a0*a42);
  a38=(a38+a49);
  a49=arg[1] ? arg[1][14] : 0;
  a50=(a49*a48);
  a38=(a38+a50);
  a50=arg[1] ? arg[1][15] : 0;
  a51=(a50*a46);
  a38=(a38-a51);
  if (res[1]!=0) res[1][1]=a38;
  a38=arg[1] ? arg[1][18] : 0;
  a51=(a38*a34);
  a52=arg[1] ? arg[1][19] : 0;
  a53=(a52*a42);
  a51=(a51+a53);
  a53=arg[1] ? arg[1][22] : 0;
  a54=(a53*a48);
  a51=(a51+a54);
  a54=arg[1] ? arg[1][23] : 0;
  a55=(a54*a46);
  a51=(a51-a55);
  if (res[1]!=0) res[1][2]=a51;
  a51=arg[1] ? arg[1][26] : 0;
  a55=(a51*a34);
  a56=arg[1] ? arg[1][27] : 0;
  a57=(a56*a42);
  a55=(a55+a57);
  a57=arg[1] ? arg[1][30] : 0;
  a58=(a57*a48);
  a55=(a55+a58);
  a58=arg[1] ? arg[1][31] : 0;
  a59=(a58*a46);
  a55=(a55-a59);
  if (res[1]!=0) res[1][3]=a55;
  a55=arg[1] ? arg[1][34] : 0;
  a59=(a55*a34);
  a60=arg[1] ? arg[1][35] : 0;
  a61=(a60*a42);
  a59=(a59+a61);
  a61=arg[1] ? arg[1][38] : 0;
  a62=(a61*a48);
  a59=(a59+a62);
  a62=arg[1] ? arg[1][39] : 0;
  a63=(a62*a46);
  a59=(a59-a63);
  if (res[1]!=0) res[1][4]=a59;
  a59=arg[1] ? arg[1][42] : 0;
  a63=(a59*a34);
  a64=arg[1] ? arg[1][43] : 0;
  a65=(a64*a42);
  a63=(a63+a65);
  a65=arg[1] ? arg[1][46] : 0;
  a66=(a65*a48);
  a63=(a63+a66);
  a66=arg[1] ? arg[1][47] : 0;
  a67=(a66*a46);
  a63=(a63-a67);
  if (res[1]!=0) res[1][5]=a63;
  a63=arg[1] ? arg[1][50] : 0;
  a67=(a63*a34);
  a68=arg[1] ? arg[1][51] : 0;
  a69=(a68*a42);
  a67=(a67+a69);
  a69=arg[1] ? arg[1][54] : 0;
  a70=(a69*a48);
  a67=(a67+a70);
  a70=arg[1] ? arg[1][55] : 0;
  a71=(a70*a46);
  a67=(a67-a71);
  if (res[1]!=0) res[1][6]=a67;
  a67=arg[1] ? arg[1][58] : 0;
  a71=(a67*a34);
  a72=arg[1] ? arg[1][59] : 0;
  a73=(a72*a42);
  a71=(a71+a73);
  a73=arg[1] ? arg[1][62] : 0;
  a74=(a73*a48);
  a71=(a71+a74);
  a74=arg[1] ? arg[1][63] : 0;
  a75=(a74*a46);
  a71=(a71-a75);
  if (res[1]!=0) res[1][7]=a71;
  a71=arg[2] ? arg[2][2] : 0;
  a75=(a71*a34);
  a76=arg[2] ? arg[2][3] : 0;
  a77=(a76*a42);
  a75=(a75+a77);
  a77=arg[2] ? arg[2][6] : 0;
  a78=(a77*a48);
  a75=(a75+a78);
  a78=arg[2] ? arg[2][7] : 0;
  a79=(a78*a46);
  a75=(a75-a79);
  a7=(a7/a29);
  a47=(a7*a47);
  a47=(a27*a47);
  a75=(a75-a47);
  if (res[1]!=0) res[1][8]=a75;
  a75=arg[2] ? arg[2][10] : 0;
  a79=(a75*a34);
  a80=arg[2] ? arg[2][11] : 0;
  a81=(a80*a42);
  a79=(a79+a81);
  a81=arg[2] ? arg[2][14] : 0;
  a82=(a81*a48);
  a79=(a79+a82);
  a82=arg[2] ? arg[2][15] : 0;
  a83=(a82*a46);
  a79=(a79-a83);
  a79=(a79+a47);
  if (res[1]!=0) res[1][9]=a79;
  a79=arg[2] ? arg[2][18] : 0;
  a47=(a79*a34);
  a83=arg[2] ? arg[2][19] : 0;
  a84=(a83*a42);
  a47=(a47+a84);
  a84=arg[2] ? arg[2][22] : 0;
  a85=(a84*a48);
  a47=(a47+a85);
  a85=arg[2] ? arg[2][23] : 0;
  a86=(a85*a46);
  a47=(a47-a86);
  if (res[1]!=0) res[1][10]=a47;
  a47=arg[2] ? arg[2][26] : 0;
  a86=(a47*a34);
  a87=arg[2] ? arg[2][27] : 0;
  a88=(a87*a42);
  a86=(a86+a88);
  a88=arg[2] ? arg[2][30] : 0;
  a89=(a88*a48);
  a86=(a86+a89);
  a89=arg[2] ? arg[2][31] : 0;
  a90=(a89*a46);
  a86=(a86-a90);
  if (res[1]!=0) res[1][11]=a86;
  a86=arg[2] ? arg[2][34] : 0;
  a34=(a86*a34);
  a90=arg[2] ? arg[2][35] : 0;
  a42=(a90*a42);
  a34=(a34+a42);
  a42=arg[2] ? arg[2][38] : 0;
  a48=(a42*a48);
  a34=(a34+a48);
  a48=arg[2] ? arg[2][39] : 0;
  a46=(a48*a46);
  a34=(a34-a46);
  if (res[1]!=0) res[1][12]=a34;
  a34=(a5*a49);
  a34=(a3*a34);
  a34=(a2*a34);
  a46=(a35*a40);
  a46=(a6*a46);
  a34=(a34-a46);
  a46=(a36*a40);
  a46=(a11*a46);
  a91=(a5*a50);
  a91=(a9*a91);
  a91=(a8*a91);
  a46=(a46+a91);
  a34=(a34-a46);
  a46=(a13*a50);
  a46=(a9*a46);
  a46=(a12*a46);
  a91=(a39*a40);
  a91=(a10*a91);
  a46=(a46-a91);
  a34=(a34-a46);
  a46=(a37*a40);
  a46=(a15*a46);
  a91=(a13*a49);
  a91=(a3*a91);
  a91=(a14*a91);
  a46=(a46+a91);
  a34=(a34-a46);
  a46=(a40*a34);
  a91=(a19*a49);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a0);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a0);
  a92=(a33*a92);
  a93=(a21*a0);
  a94=(a25*a93);
  a95=(a28*a94);
  a95=(a95/a29);
  a96=(a30*a94);
  a95=(a95+a96);
  a95=(a31*a95);
  a95=(a26*a95);
  a93=(a93+a93);
  a93=(a32*a93);
  a95=(a95-a93);
  a95=(a24*a95);
  a92=(a92+a95);
  a91=(a91-a92);
  a92=(a0*a91);
  a46=(a46+a92);
  a92=(a45*a40);
  a92=(a3*a92);
  a92=(a5*a92);
  a0=(a0/a20);
  a0=(a17*a0);
  a0=(a18*a0);
  a0=(a19*a0);
  a92=(a92+a0);
  a0=(a41*a40);
  a0=(a3*a0);
  a0=(a13*a0);
  a92=(a92-a0);
  a49=(a49*a92);
  a46=(a46+a49);
  a49=(a16*a40);
  a49=(a9*a49);
  a49=(a5*a49);
  a40=(a1*a40);
  a40=(a9*a40);
  a40=(a13*a40);
  a49=(a49+a40);
  a50=(a50*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][13]=a46;
  a46=(a38*a34);
  a50=(a52*a91);
  a46=(a46+a50);
  a50=(a53*a92);
  a46=(a46+a50);
  a50=(a54*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][14]=a46;
  a46=(a51*a34);
  a50=(a56*a91);
  a46=(a46+a50);
  a50=(a57*a92);
  a46=(a46+a50);
  a50=(a58*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][15]=a46;
  a46=(a55*a34);
  a50=(a60*a91);
  a46=(a46+a50);
  a50=(a61*a92);
  a46=(a46+a50);
  a50=(a62*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][16]=a46;
  a46=(a59*a34);
  a50=(a64*a91);
  a46=(a46+a50);
  a50=(a65*a92);
  a46=(a46+a50);
  a50=(a66*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][17]=a46;
  a46=(a63*a34);
  a50=(a68*a91);
  a46=(a46+a50);
  a50=(a69*a92);
  a46=(a46+a50);
  a50=(a70*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][18]=a46;
  a46=(a67*a34);
  a50=(a72*a91);
  a46=(a46+a50);
  a50=(a73*a92);
  a46=(a46+a50);
  a50=(a74*a49);
  a46=(a46-a50);
  if (res[1]!=0) res[1][19]=a46;
  a46=(a71*a34);
  a50=(a76*a91);
  a46=(a46+a50);
  a50=(a77*a92);
  a46=(a46+a50);
  a50=(a78*a49);
  a46=(a46-a50);
  a94=(a7*a94);
  a94=(a27*a94);
  a46=(a46-a94);
  if (res[1]!=0) res[1][20]=a46;
  a46=(a75*a34);
  a50=(a80*a91);
  a46=(a46+a50);
  a50=(a81*a92);
  a46=(a46+a50);
  a50=(a82*a49);
  a46=(a46-a50);
  a46=(a46+a94);
  if (res[1]!=0) res[1][21]=a46;
  a46=(a79*a34);
  a94=(a83*a91);
  a46=(a46+a94);
  a94=(a84*a92);
  a46=(a46+a94);
  a94=(a85*a49);
  a46=(a46-a94);
  if (res[1]!=0) res[1][22]=a46;
  a46=(a47*a34);
  a94=(a87*a91);
  a46=(a46+a94);
  a94=(a88*a92);
  a46=(a46+a94);
  a94=(a89*a49);
  a46=(a46-a94);
  if (res[1]!=0) res[1][23]=a46;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a49=(a48*a49);
  a34=(a34-a49);
  if (res[1]!=0) res[1][24]=a34;
  a34=(a5*a53);
  a34=(a3*a34);
  a34=(a2*a34);
  a49=(a35*a38);
  a49=(a6*a49);
  a34=(a34-a49);
  a49=(a36*a38);
  a49=(a11*a49);
  a92=(a5*a54);
  a92=(a9*a92);
  a92=(a8*a92);
  a49=(a49+a92);
  a34=(a34-a49);
  a49=(a13*a54);
  a49=(a9*a49);
  a49=(a12*a49);
  a92=(a39*a38);
  a92=(a10*a92);
  a49=(a49-a92);
  a34=(a34-a49);
  a49=(a37*a38);
  a49=(a15*a49);
  a92=(a13*a53);
  a92=(a3*a92);
  a92=(a14*a92);
  a49=(a49+a92);
  a34=(a34-a49);
  a49=(a38*a34);
  a92=(a19*a53);
  a92=(a18*a92);
  a92=(a17*a92);
  a92=(a92/a23);
  a91=(a43*a52);
  a91=(a22*a91);
  a91=(a4*a91);
  a92=(a92-a91);
  a91=(a44*a52);
  a91=(a33*a91);
  a46=(a21*a52);
  a94=(a25*a46);
  a50=(a28*a94);
  a50=(a50/a29);
  a40=(a30*a94);
  a50=(a50+a40);
  a50=(a31*a50);
  a50=(a26*a50);
  a46=(a46+a46);
  a46=(a32*a46);
  a50=(a50-a46);
  a50=(a24*a50);
  a91=(a91+a50);
  a92=(a92-a91);
  a91=(a52*a92);
  a49=(a49+a91);
  a91=(a45*a38);
  a91=(a3*a91);
  a91=(a5*a91);
  a52=(a52/a20);
  a52=(a17*a52);
  a52=(a18*a52);
  a52=(a19*a52);
  a91=(a91+a52);
  a52=(a41*a38);
  a52=(a3*a52);
  a52=(a13*a52);
  a91=(a91-a52);
  a53=(a53*a91);
  a49=(a49+a53);
  a53=(a16*a38);
  a53=(a9*a53);
  a53=(a5*a53);
  a38=(a1*a38);
  a38=(a9*a38);
  a38=(a13*a38);
  a53=(a53+a38);
  a54=(a54*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][25]=a49;
  a49=(a51*a34);
  a54=(a56*a92);
  a49=(a49+a54);
  a54=(a57*a91);
  a49=(a49+a54);
  a54=(a58*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][26]=a49;
  a49=(a55*a34);
  a54=(a60*a92);
  a49=(a49+a54);
  a54=(a61*a91);
  a49=(a49+a54);
  a54=(a62*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][27]=a49;
  a49=(a59*a34);
  a54=(a64*a92);
  a49=(a49+a54);
  a54=(a65*a91);
  a49=(a49+a54);
  a54=(a66*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][28]=a49;
  a49=(a63*a34);
  a54=(a68*a92);
  a49=(a49+a54);
  a54=(a69*a91);
  a49=(a49+a54);
  a54=(a70*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][29]=a49;
  a49=(a67*a34);
  a54=(a72*a92);
  a49=(a49+a54);
  a54=(a73*a91);
  a49=(a49+a54);
  a54=(a74*a53);
  a49=(a49-a54);
  if (res[1]!=0) res[1][30]=a49;
  a49=(a71*a34);
  a54=(a76*a92);
  a49=(a49+a54);
  a54=(a77*a91);
  a49=(a49+a54);
  a54=(a78*a53);
  a49=(a49-a54);
  a94=(a7*a94);
  a94=(a27*a94);
  a49=(a49-a94);
  if (res[1]!=0) res[1][31]=a49;
  a49=(a75*a34);
  a54=(a80*a92);
  a49=(a49+a54);
  a54=(a81*a91);
  a49=(a49+a54);
  a54=(a82*a53);
  a49=(a49-a54);
  a49=(a49+a94);
  if (res[1]!=0) res[1][32]=a49;
  a49=(a79*a34);
  a94=(a83*a92);
  a49=(a49+a94);
  a94=(a84*a91);
  a49=(a49+a94);
  a94=(a85*a53);
  a49=(a49-a94);
  if (res[1]!=0) res[1][33]=a49;
  a49=(a47*a34);
  a94=(a87*a92);
  a49=(a49+a94);
  a94=(a88*a91);
  a49=(a49+a94);
  a94=(a89*a53);
  a49=(a49-a94);
  if (res[1]!=0) res[1][34]=a49;
  a34=(a86*a34);
  a92=(a90*a92);
  a34=(a34+a92);
  a91=(a42*a91);
  a34=(a34+a91);
  a53=(a48*a53);
  a34=(a34-a53);
  if (res[1]!=0) res[1][35]=a34;
  a34=(a5*a57);
  a34=(a3*a34);
  a34=(a2*a34);
  a53=(a35*a51);
  a53=(a6*a53);
  a34=(a34-a53);
  a53=(a36*a51);
  a53=(a11*a53);
  a91=(a5*a58);
  a91=(a9*a91);
  a91=(a8*a91);
  a53=(a53+a91);
  a34=(a34-a53);
  a53=(a13*a58);
  a53=(a9*a53);
  a53=(a12*a53);
  a91=(a39*a51);
  a91=(a10*a91);
  a53=(a53-a91);
  a34=(a34-a53);
  a53=(a37*a51);
  a53=(a15*a53);
  a91=(a13*a57);
  a91=(a3*a91);
  a91=(a14*a91);
  a53=(a53+a91);
  a34=(a34-a53);
  a53=(a51*a34);
  a91=(a19*a57);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a56);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a56);
  a92=(a33*a92);
  a49=(a21*a56);
  a94=(a25*a49);
  a54=(a28*a94);
  a54=(a54/a29);
  a38=(a30*a94);
  a54=(a54+a38);
  a54=(a31*a54);
  a54=(a26*a54);
  a49=(a49+a49);
  a49=(a32*a49);
  a54=(a54-a49);
  a54=(a24*a54);
  a92=(a92+a54);
  a91=(a91-a92);
  a92=(a56*a91);
  a53=(a53+a92);
  a92=(a45*a51);
  a92=(a3*a92);
  a92=(a5*a92);
  a56=(a56/a20);
  a56=(a17*a56);
  a56=(a18*a56);
  a56=(a19*a56);
  a92=(a92+a56);
  a56=(a41*a51);
  a56=(a3*a56);
  a56=(a13*a56);
  a92=(a92-a56);
  a57=(a57*a92);
  a53=(a53+a57);
  a57=(a16*a51);
  a57=(a9*a57);
  a57=(a5*a57);
  a51=(a1*a51);
  a51=(a9*a51);
  a51=(a13*a51);
  a57=(a57+a51);
  a58=(a58*a57);
  a53=(a53-a58);
  if (res[1]!=0) res[1][36]=a53;
  a53=(a55*a34);
  a58=(a60*a91);
  a53=(a53+a58);
  a58=(a61*a92);
  a53=(a53+a58);
  a58=(a62*a57);
  a53=(a53-a58);
  if (res[1]!=0) res[1][37]=a53;
  a53=(a59*a34);
  a58=(a64*a91);
  a53=(a53+a58);
  a58=(a65*a92);
  a53=(a53+a58);
  a58=(a66*a57);
  a53=(a53-a58);
  if (res[1]!=0) res[1][38]=a53;
  a53=(a63*a34);
  a58=(a68*a91);
  a53=(a53+a58);
  a58=(a69*a92);
  a53=(a53+a58);
  a58=(a70*a57);
  a53=(a53-a58);
  if (res[1]!=0) res[1][39]=a53;
  a53=(a67*a34);
  a58=(a72*a91);
  a53=(a53+a58);
  a58=(a73*a92);
  a53=(a53+a58);
  a58=(a74*a57);
  a53=(a53-a58);
  if (res[1]!=0) res[1][40]=a53;
  a53=(a71*a34);
  a58=(a76*a91);
  a53=(a53+a58);
  a58=(a77*a92);
  a53=(a53+a58);
  a58=(a78*a57);
  a53=(a53-a58);
  a94=(a7*a94);
  a94=(a27*a94);
  a53=(a53-a94);
  if (res[1]!=0) res[1][41]=a53;
  a53=(a75*a34);
  a58=(a80*a91);
  a53=(a53+a58);
  a58=(a81*a92);
  a53=(a53+a58);
  a58=(a82*a57);
  a53=(a53-a58);
  a53=(a53+a94);
  if (res[1]!=0) res[1][42]=a53;
  a53=(a79*a34);
  a94=(a83*a91);
  a53=(a53+a94);
  a94=(a84*a92);
  a53=(a53+a94);
  a94=(a85*a57);
  a53=(a53-a94);
  if (res[1]!=0) res[1][43]=a53;
  a53=(a47*a34);
  a94=(a87*a91);
  a53=(a53+a94);
  a94=(a88*a92);
  a53=(a53+a94);
  a94=(a89*a57);
  a53=(a53-a94);
  if (res[1]!=0) res[1][44]=a53;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a57=(a48*a57);
  a34=(a34-a57);
  if (res[1]!=0) res[1][45]=a34;
  a34=(a5*a61);
  a34=(a3*a34);
  a34=(a2*a34);
  a57=(a35*a55);
  a57=(a6*a57);
  a34=(a34-a57);
  a57=(a36*a55);
  a57=(a11*a57);
  a92=(a5*a62);
  a92=(a9*a92);
  a92=(a8*a92);
  a57=(a57+a92);
  a34=(a34-a57);
  a57=(a13*a62);
  a57=(a9*a57);
  a57=(a12*a57);
  a92=(a39*a55);
  a92=(a10*a92);
  a57=(a57-a92);
  a34=(a34-a57);
  a57=(a37*a55);
  a57=(a15*a57);
  a92=(a13*a61);
  a92=(a3*a92);
  a92=(a14*a92);
  a57=(a57+a92);
  a34=(a34-a57);
  a57=(a55*a34);
  a92=(a19*a61);
  a92=(a18*a92);
  a92=(a17*a92);
  a92=(a92/a23);
  a91=(a43*a60);
  a91=(a22*a91);
  a91=(a4*a91);
  a92=(a92-a91);
  a91=(a44*a60);
  a91=(a33*a91);
  a53=(a21*a60);
  a94=(a25*a53);
  a58=(a28*a94);
  a58=(a58/a29);
  a51=(a30*a94);
  a58=(a58+a51);
  a58=(a31*a58);
  a58=(a26*a58);
  a53=(a53+a53);
  a53=(a32*a53);
  a58=(a58-a53);
  a58=(a24*a58);
  a91=(a91+a58);
  a92=(a92-a91);
  a91=(a60*a92);
  a57=(a57+a91);
  a91=(a45*a55);
  a91=(a3*a91);
  a91=(a5*a91);
  a60=(a60/a20);
  a60=(a17*a60);
  a60=(a18*a60);
  a60=(a19*a60);
  a91=(a91+a60);
  a60=(a41*a55);
  a60=(a3*a60);
  a60=(a13*a60);
  a91=(a91-a60);
  a61=(a61*a91);
  a57=(a57+a61);
  a61=(a16*a55);
  a61=(a9*a61);
  a61=(a5*a61);
  a55=(a1*a55);
  a55=(a9*a55);
  a55=(a13*a55);
  a61=(a61+a55);
  a62=(a62*a61);
  a57=(a57-a62);
  if (res[1]!=0) res[1][46]=a57;
  a57=(a59*a34);
  a62=(a64*a92);
  a57=(a57+a62);
  a62=(a65*a91);
  a57=(a57+a62);
  a62=(a66*a61);
  a57=(a57-a62);
  if (res[1]!=0) res[1][47]=a57;
  a57=(a63*a34);
  a62=(a68*a92);
  a57=(a57+a62);
  a62=(a69*a91);
  a57=(a57+a62);
  a62=(a70*a61);
  a57=(a57-a62);
  if (res[1]!=0) res[1][48]=a57;
  a57=(a67*a34);
  a62=(a72*a92);
  a57=(a57+a62);
  a62=(a73*a91);
  a57=(a57+a62);
  a62=(a74*a61);
  a57=(a57-a62);
  if (res[1]!=0) res[1][49]=a57;
  a57=(a71*a34);
  a62=(a76*a92);
  a57=(a57+a62);
  a62=(a77*a91);
  a57=(a57+a62);
  a62=(a78*a61);
  a57=(a57-a62);
  a94=(a7*a94);
  a94=(a27*a94);
  a57=(a57-a94);
  if (res[1]!=0) res[1][50]=a57;
  a57=(a75*a34);
  a62=(a80*a92);
  a57=(a57+a62);
  a62=(a81*a91);
  a57=(a57+a62);
  a62=(a82*a61);
  a57=(a57-a62);
  a57=(a57+a94);
  if (res[1]!=0) res[1][51]=a57;
  a57=(a79*a34);
  a94=(a83*a92);
  a57=(a57+a94);
  a94=(a84*a91);
  a57=(a57+a94);
  a94=(a85*a61);
  a57=(a57-a94);
  if (res[1]!=0) res[1][52]=a57;
  a57=(a47*a34);
  a94=(a87*a92);
  a57=(a57+a94);
  a94=(a88*a91);
  a57=(a57+a94);
  a94=(a89*a61);
  a57=(a57-a94);
  if (res[1]!=0) res[1][53]=a57;
  a34=(a86*a34);
  a92=(a90*a92);
  a34=(a34+a92);
  a91=(a42*a91);
  a34=(a34+a91);
  a61=(a48*a61);
  a34=(a34-a61);
  if (res[1]!=0) res[1][54]=a34;
  a34=(a5*a65);
  a34=(a3*a34);
  a34=(a2*a34);
  a61=(a35*a59);
  a61=(a6*a61);
  a34=(a34-a61);
  a61=(a36*a59);
  a61=(a11*a61);
  a91=(a5*a66);
  a91=(a9*a91);
  a91=(a8*a91);
  a61=(a61+a91);
  a34=(a34-a61);
  a61=(a13*a66);
  a61=(a9*a61);
  a61=(a12*a61);
  a91=(a39*a59);
  a91=(a10*a91);
  a61=(a61-a91);
  a34=(a34-a61);
  a61=(a37*a59);
  a61=(a15*a61);
  a91=(a13*a65);
  a91=(a3*a91);
  a91=(a14*a91);
  a61=(a61+a91);
  a34=(a34-a61);
  a61=(a59*a34);
  a91=(a19*a65);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a64);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a64);
  a92=(a33*a92);
  a57=(a21*a64);
  a94=(a25*a57);
  a62=(a28*a94);
  a62=(a62/a29);
  a55=(a30*a94);
  a62=(a62+a55);
  a62=(a31*a62);
  a62=(a26*a62);
  a57=(a57+a57);
  a57=(a32*a57);
  a62=(a62-a57);
  a62=(a24*a62);
  a92=(a92+a62);
  a91=(a91-a92);
  a92=(a64*a91);
  a61=(a61+a92);
  a92=(a45*a59);
  a92=(a3*a92);
  a92=(a5*a92);
  a64=(a64/a20);
  a64=(a17*a64);
  a64=(a18*a64);
  a64=(a19*a64);
  a92=(a92+a64);
  a64=(a41*a59);
  a64=(a3*a64);
  a64=(a13*a64);
  a92=(a92-a64);
  a65=(a65*a92);
  a61=(a61+a65);
  a65=(a16*a59);
  a65=(a9*a65);
  a65=(a5*a65);
  a59=(a1*a59);
  a59=(a9*a59);
  a59=(a13*a59);
  a65=(a65+a59);
  a66=(a66*a65);
  a61=(a61-a66);
  if (res[1]!=0) res[1][55]=a61;
  a61=(a63*a34);
  a66=(a68*a91);
  a61=(a61+a66);
  a66=(a69*a92);
  a61=(a61+a66);
  a66=(a70*a65);
  a61=(a61-a66);
  if (res[1]!=0) res[1][56]=a61;
  a61=(a67*a34);
  a66=(a72*a91);
  a61=(a61+a66);
  a66=(a73*a92);
  a61=(a61+a66);
  a66=(a74*a65);
  a61=(a61-a66);
  if (res[1]!=0) res[1][57]=a61;
  a61=(a71*a34);
  a66=(a76*a91);
  a61=(a61+a66);
  a66=(a77*a92);
  a61=(a61+a66);
  a66=(a78*a65);
  a61=(a61-a66);
  a94=(a7*a94);
  a94=(a27*a94);
  a61=(a61-a94);
  if (res[1]!=0) res[1][58]=a61;
  a61=(a75*a34);
  a66=(a80*a91);
  a61=(a61+a66);
  a66=(a81*a92);
  a61=(a61+a66);
  a66=(a82*a65);
  a61=(a61-a66);
  a61=(a61+a94);
  if (res[1]!=0) res[1][59]=a61;
  a61=(a79*a34);
  a94=(a83*a91);
  a61=(a61+a94);
  a94=(a84*a92);
  a61=(a61+a94);
  a94=(a85*a65);
  a61=(a61-a94);
  if (res[1]!=0) res[1][60]=a61;
  a61=(a47*a34);
  a94=(a87*a91);
  a61=(a61+a94);
  a94=(a88*a92);
  a61=(a61+a94);
  a94=(a89*a65);
  a61=(a61-a94);
  if (res[1]!=0) res[1][61]=a61;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a65=(a48*a65);
  a34=(a34-a65);
  if (res[1]!=0) res[1][62]=a34;
  a34=(a5*a69);
  a34=(a3*a34);
  a34=(a2*a34);
  a65=(a35*a63);
  a65=(a6*a65);
  a34=(a34-a65);
  a65=(a36*a63);
  a65=(a11*a65);
  a92=(a5*a70);
  a92=(a9*a92);
  a92=(a8*a92);
  a65=(a65+a92);
  a34=(a34-a65);
  a65=(a13*a70);
  a65=(a9*a65);
  a65=(a12*a65);
  a92=(a39*a63);
  a92=(a10*a92);
  a65=(a65-a92);
  a34=(a34-a65);
  a65=(a37*a63);
  a65=(a15*a65);
  a92=(a13*a69);
  a92=(a3*a92);
  a92=(a14*a92);
  a65=(a65+a92);
  a34=(a34-a65);
  a65=(a63*a34);
  a92=(a19*a69);
  a92=(a18*a92);
  a92=(a17*a92);
  a92=(a92/a23);
  a91=(a43*a68);
  a91=(a22*a91);
  a91=(a4*a91);
  a92=(a92-a91);
  a91=(a44*a68);
  a91=(a33*a91);
  a61=(a21*a68);
  a94=(a25*a61);
  a66=(a28*a94);
  a66=(a66/a29);
  a59=(a30*a94);
  a66=(a66+a59);
  a66=(a31*a66);
  a66=(a26*a66);
  a61=(a61+a61);
  a61=(a32*a61);
  a66=(a66-a61);
  a66=(a24*a66);
  a91=(a91+a66);
  a92=(a92-a91);
  a91=(a68*a92);
  a65=(a65+a91);
  a91=(a45*a63);
  a91=(a3*a91);
  a91=(a5*a91);
  a68=(a68/a20);
  a68=(a17*a68);
  a68=(a18*a68);
  a68=(a19*a68);
  a91=(a91+a68);
  a68=(a41*a63);
  a68=(a3*a68);
  a68=(a13*a68);
  a91=(a91-a68);
  a69=(a69*a91);
  a65=(a65+a69);
  a69=(a16*a63);
  a69=(a9*a69);
  a69=(a5*a69);
  a63=(a1*a63);
  a63=(a9*a63);
  a63=(a13*a63);
  a69=(a69+a63);
  a70=(a70*a69);
  a65=(a65-a70);
  if (res[1]!=0) res[1][63]=a65;
  a65=(a67*a34);
  a70=(a72*a92);
  a65=(a65+a70);
  a70=(a73*a91);
  a65=(a65+a70);
  a70=(a74*a69);
  a65=(a65-a70);
  if (res[1]!=0) res[1][64]=a65;
  a65=(a71*a34);
  a70=(a76*a92);
  a65=(a65+a70);
  a70=(a77*a91);
  a65=(a65+a70);
  a70=(a78*a69);
  a65=(a65-a70);
  a94=(a7*a94);
  a94=(a27*a94);
  a65=(a65-a94);
  if (res[1]!=0) res[1][65]=a65;
  a65=(a75*a34);
  a70=(a80*a92);
  a65=(a65+a70);
  a70=(a81*a91);
  a65=(a65+a70);
  a70=(a82*a69);
  a65=(a65-a70);
  a65=(a65+a94);
  if (res[1]!=0) res[1][66]=a65;
  a65=(a79*a34);
  a94=(a83*a92);
  a65=(a65+a94);
  a94=(a84*a91);
  a65=(a65+a94);
  a94=(a85*a69);
  a65=(a65-a94);
  if (res[1]!=0) res[1][67]=a65;
  a65=(a47*a34);
  a94=(a87*a92);
  a65=(a65+a94);
  a94=(a88*a91);
  a65=(a65+a94);
  a94=(a89*a69);
  a65=(a65-a94);
  if (res[1]!=0) res[1][68]=a65;
  a34=(a86*a34);
  a92=(a90*a92);
  a34=(a34+a92);
  a91=(a42*a91);
  a34=(a34+a91);
  a69=(a48*a69);
  a34=(a34-a69);
  if (res[1]!=0) res[1][69]=a34;
  a34=(a5*a73);
  a34=(a3*a34);
  a34=(a2*a34);
  a69=(a35*a67);
  a69=(a6*a69);
  a34=(a34-a69);
  a69=(a36*a67);
  a69=(a11*a69);
  a91=(a5*a74);
  a91=(a9*a91);
  a91=(a8*a91);
  a69=(a69+a91);
  a34=(a34-a69);
  a69=(a13*a74);
  a69=(a9*a69);
  a69=(a12*a69);
  a91=(a39*a67);
  a91=(a10*a91);
  a69=(a69-a91);
  a34=(a34-a69);
  a69=(a37*a67);
  a69=(a15*a69);
  a91=(a13*a73);
  a91=(a3*a91);
  a91=(a14*a91);
  a69=(a69+a91);
  a34=(a34-a69);
  a69=(a67*a34);
  a91=(a19*a73);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a72);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a72);
  a92=(a33*a92);
  a65=(a21*a72);
  a94=(a25*a65);
  a70=(a28*a94);
  a70=(a70/a29);
  a63=(a30*a94);
  a70=(a70+a63);
  a70=(a31*a70);
  a70=(a26*a70);
  a65=(a65+a65);
  a65=(a32*a65);
  a70=(a70-a65);
  a70=(a24*a70);
  a92=(a92+a70);
  a91=(a91-a92);
  a92=(a72*a91);
  a69=(a69+a92);
  a92=(a45*a67);
  a92=(a3*a92);
  a92=(a5*a92);
  a72=(a72/a20);
  a72=(a17*a72);
  a72=(a18*a72);
  a72=(a19*a72);
  a92=(a92+a72);
  a72=(a41*a67);
  a72=(a3*a72);
  a72=(a13*a72);
  a92=(a92-a72);
  a73=(a73*a92);
  a69=(a69+a73);
  a73=(a16*a67);
  a73=(a9*a73);
  a73=(a5*a73);
  a67=(a1*a67);
  a67=(a9*a67);
  a67=(a13*a67);
  a73=(a73+a67);
  a74=(a74*a73);
  a69=(a69-a74);
  if (res[1]!=0) res[1][70]=a69;
  a69=(a71*a34);
  a74=(a76*a91);
  a69=(a69+a74);
  a74=(a77*a92);
  a69=(a69+a74);
  a74=(a78*a73);
  a69=(a69-a74);
  a94=(a7*a94);
  a94=(a27*a94);
  a69=(a69-a94);
  if (res[1]!=0) res[1][71]=a69;
  a69=(a75*a34);
  a74=(a80*a91);
  a69=(a69+a74);
  a74=(a81*a92);
  a69=(a69+a74);
  a74=(a82*a73);
  a69=(a69-a74);
  a69=(a69+a94);
  if (res[1]!=0) res[1][72]=a69;
  a69=(a79*a34);
  a94=(a83*a91);
  a69=(a69+a94);
  a94=(a84*a92);
  a69=(a69+a94);
  a94=(a85*a73);
  a69=(a69-a94);
  if (res[1]!=0) res[1][73]=a69;
  a69=(a47*a34);
  a94=(a87*a91);
  a69=(a69+a94);
  a94=(a88*a92);
  a69=(a69+a94);
  a94=(a89*a73);
  a69=(a69-a94);
  if (res[1]!=0) res[1][74]=a69;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a73=(a48*a73);
  a34=(a34-a73);
  if (res[1]!=0) res[1][75]=a34;
  a34=(a5*a77);
  a34=(a3*a34);
  a34=(a2*a34);
  a73=(a35*a71);
  a73=(a6*a73);
  a34=(a34-a73);
  a73=(a36*a71);
  a73=(a11*a73);
  a92=(a5*a78);
  a92=(a9*a92);
  a92=(a8*a92);
  a73=(a73+a92);
  a34=(a34-a73);
  a73=(a13*a78);
  a73=(a9*a73);
  a73=(a12*a73);
  a92=(a39*a71);
  a92=(a10*a92);
  a73=(a73-a92);
  a34=(a34-a73);
  a73=(a37*a71);
  a73=(a15*a73);
  a92=(a13*a77);
  a92=(a3*a92);
  a92=(a14*a92);
  a73=(a73+a92);
  a34=(a34-a73);
  a73=(a71*a34);
  a92=(a19*a77);
  a92=(a18*a92);
  a92=(a17*a92);
  a92=(a92/a23);
  a91=(a43*a76);
  a91=(a22*a91);
  a91=(a4*a91);
  a92=(a92-a91);
  a91=(a44*a76);
  a91=(a33*a91);
  a69=(a27/a29);
  a94=(a21*a76);
  a74=(a25*a94);
  a67=(a28*a74);
  a69=(a69+a67);
  a69=(a69/a29);
  a67=(a30*a74);
  a69=(a69+a67);
  a69=(a31*a69);
  a69=(a26*a69);
  a94=(a94+a94);
  a94=(a32*a94);
  a69=(a69-a94);
  a69=(a24*a69);
  a91=(a91+a69);
  a92=(a92-a91);
  a91=(a76*a92);
  a73=(a73+a91);
  a91=(a45*a71);
  a91=(a3*a91);
  a91=(a5*a91);
  a76=(a76/a20);
  a76=(a17*a76);
  a76=(a18*a76);
  a76=(a19*a76);
  a91=(a91+a76);
  a76=(a41*a71);
  a76=(a3*a76);
  a76=(a13*a76);
  a91=(a91-a76);
  a77=(a77*a91);
  a73=(a73+a77);
  a77=(a16*a71);
  a77=(a9*a77);
  a77=(a5*a77);
  a71=(a1*a71);
  a71=(a9*a71);
  a71=(a13*a71);
  a77=(a77+a71);
  a78=(a78*a77);
  a73=(a73-a78);
  a74=(a7*a74);
  a74=(a27*a74);
  a73=(a73-a74);
  if (res[1]!=0) res[1][76]=a73;
  a73=(a75*a34);
  a78=(a80*a92);
  a73=(a73+a78);
  a78=(a81*a91);
  a73=(a73+a78);
  a78=(a82*a77);
  a73=(a73-a78);
  a73=(a73+a74);
  if (res[1]!=0) res[1][77]=a73;
  a73=(a79*a34);
  a74=(a83*a92);
  a73=(a73+a74);
  a74=(a84*a91);
  a73=(a73+a74);
  a74=(a85*a77);
  a73=(a73-a74);
  if (res[1]!=0) res[1][78]=a73;
  a73=(a47*a34);
  a74=(a87*a92);
  a73=(a73+a74);
  a74=(a88*a91);
  a73=(a73+a74);
  a74=(a89*a77);
  a73=(a73-a74);
  if (res[1]!=0) res[1][79]=a73;
  a34=(a86*a34);
  a92=(a90*a92);
  a34=(a34+a92);
  a91=(a42*a91);
  a34=(a34+a91);
  a77=(a48*a77);
  a34=(a34-a77);
  if (res[1]!=0) res[1][80]=a34;
  a34=(a5*a81);
  a34=(a3*a34);
  a34=(a2*a34);
  a77=(a35*a75);
  a77=(a6*a77);
  a34=(a34-a77);
  a77=(a36*a75);
  a77=(a11*a77);
  a91=(a5*a82);
  a91=(a9*a91);
  a91=(a8*a91);
  a77=(a77+a91);
  a34=(a34-a77);
  a77=(a13*a82);
  a77=(a9*a77);
  a77=(a12*a77);
  a91=(a39*a75);
  a91=(a10*a91);
  a77=(a77-a91);
  a34=(a34-a77);
  a77=(a37*a75);
  a77=(a15*a77);
  a91=(a13*a81);
  a91=(a3*a91);
  a91=(a14*a91);
  a77=(a77+a91);
  a34=(a34-a77);
  a77=(a75*a34);
  a91=(a19*a81);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a80);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a80);
  a92=(a33*a92);
  a73=(a21*a80);
  a74=(a25*a73);
  a78=(a28*a74);
  a71=(a27/a29);
  a78=(a78-a71);
  a78=(a78/a29);
  a71=(a30*a74);
  a78=(a78+a71);
  a78=(a31*a78);
  a78=(a26*a78);
  a73=(a73+a73);
  a73=(a32*a73);
  a78=(a78-a73);
  a78=(a24*a78);
  a92=(a92+a78);
  a91=(a91-a92);
  a92=(a80*a91);
  a77=(a77+a92);
  a92=(a45*a75);
  a92=(a3*a92);
  a92=(a5*a92);
  a80=(a80/a20);
  a80=(a17*a80);
  a80=(a18*a80);
  a80=(a19*a80);
  a92=(a92+a80);
  a80=(a41*a75);
  a80=(a3*a80);
  a80=(a13*a80);
  a92=(a92-a80);
  a81=(a81*a92);
  a77=(a77+a81);
  a81=(a16*a75);
  a81=(a9*a81);
  a81=(a5*a81);
  a75=(a1*a75);
  a75=(a9*a75);
  a75=(a13*a75);
  a81=(a81+a75);
  a82=(a82*a81);
  a77=(a77-a82);
  a7=(a7*a74);
  a27=(a27*a7);
  a77=(a77+a27);
  if (res[1]!=0) res[1][81]=a77;
  a77=(a79*a34);
  a27=(a83*a91);
  a77=(a77+a27);
  a27=(a84*a92);
  a77=(a77+a27);
  a27=(a85*a81);
  a77=(a77-a27);
  if (res[1]!=0) res[1][82]=a77;
  a77=(a47*a34);
  a27=(a87*a91);
  a77=(a77+a27);
  a27=(a88*a92);
  a77=(a77+a27);
  a27=(a89*a81);
  a77=(a77-a27);
  if (res[1]!=0) res[1][83]=a77;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a81=(a48*a81);
  a34=(a34-a81);
  if (res[1]!=0) res[1][84]=a34;
  a34=(a5*a84);
  a34=(a3*a34);
  a34=(a2*a34);
  a81=(a35*a79);
  a81=(a6*a81);
  a34=(a34-a81);
  a81=(a36*a79);
  a81=(a11*a81);
  a92=(a5*a85);
  a92=(a9*a92);
  a92=(a8*a92);
  a81=(a81+a92);
  a34=(a34-a81);
  a81=(a13*a85);
  a81=(a9*a81);
  a81=(a12*a81);
  a92=(a39*a79);
  a92=(a10*a92);
  a81=(a81-a92);
  a34=(a34-a81);
  a81=(a37*a79);
  a81=(a15*a81);
  a92=(a13*a84);
  a92=(a3*a92);
  a92=(a14*a92);
  a81=(a81+a92);
  a34=(a34-a81);
  a81=(a79*a34);
  a92=(a19*a84);
  a92=(a18*a92);
  a92=(a17*a92);
  a92=(a92/a23);
  a91=(a43*a83);
  a91=(a22*a91);
  a91=(a4*a91);
  a92=(a92-a91);
  a91=(a44*a83);
  a91=(a33*a91);
  a77=(a21*a83);
  a27=(a25*a77);
  a7=(a28*a27);
  a7=(a7/a29);
  a27=(a30*a27);
  a7=(a7+a27);
  a7=(a31*a7);
  a7=(a26*a7);
  a77=(a77+a77);
  a77=(a32*a77);
  a7=(a7-a77);
  a7=(a24*a7);
  a91=(a91+a7);
  a92=(a92-a91);
  a91=(a83*a92);
  a81=(a81+a91);
  a91=(a45*a79);
  a91=(a3*a91);
  a91=(a5*a91);
  a83=(a83/a20);
  a83=(a17*a83);
  a83=(a18*a83);
  a83=(a19*a83);
  a91=(a91+a83);
  a83=(a41*a79);
  a83=(a3*a83);
  a83=(a13*a83);
  a91=(a91-a83);
  a84=(a84*a91);
  a81=(a81+a84);
  a84=(a16*a79);
  a84=(a9*a84);
  a84=(a5*a84);
  a79=(a1*a79);
  a79=(a9*a79);
  a79=(a13*a79);
  a84=(a84+a79);
  a85=(a85*a84);
  a81=(a81-a85);
  if (res[1]!=0) res[1][85]=a81;
  a81=(a47*a34);
  a85=(a87*a92);
  a81=(a81+a85);
  a85=(a88*a91);
  a81=(a81+a85);
  a85=(a89*a84);
  a81=(a81-a85);
  if (res[1]!=0) res[1][86]=a81;
  a34=(a86*a34);
  a92=(a90*a92);
  a34=(a34+a92);
  a91=(a42*a91);
  a34=(a34+a91);
  a84=(a48*a84);
  a34=(a34-a84);
  if (res[1]!=0) res[1][87]=a34;
  a34=(a5*a88);
  a34=(a3*a34);
  a34=(a2*a34);
  a84=(a35*a47);
  a84=(a6*a84);
  a34=(a34-a84);
  a84=(a36*a47);
  a84=(a11*a84);
  a91=(a5*a89);
  a91=(a9*a91);
  a91=(a8*a91);
  a84=(a84+a91);
  a34=(a34-a84);
  a84=(a13*a89);
  a84=(a9*a84);
  a84=(a12*a84);
  a91=(a39*a47);
  a91=(a10*a91);
  a84=(a84-a91);
  a34=(a34-a84);
  a84=(a37*a47);
  a84=(a15*a84);
  a91=(a13*a88);
  a91=(a3*a91);
  a91=(a14*a91);
  a84=(a84+a91);
  a34=(a34-a84);
  a84=(a47*a34);
  a91=(a19*a88);
  a91=(a18*a91);
  a91=(a17*a91);
  a91=(a91/a23);
  a92=(a43*a87);
  a92=(a22*a92);
  a92=(a4*a92);
  a91=(a91-a92);
  a92=(a44*a87);
  a92=(a33*a92);
  a81=(a21*a87);
  a85=(a25*a81);
  a79=(a28*a85);
  a79=(a79/a29);
  a85=(a30*a85);
  a79=(a79+a85);
  a79=(a31*a79);
  a79=(a26*a79);
  a81=(a81+a81);
  a81=(a32*a81);
  a79=(a79-a81);
  a79=(a24*a79);
  a92=(a92+a79);
  a91=(a91-a92);
  a92=(a87*a91);
  a84=(a84+a92);
  a92=(a45*a47);
  a92=(a3*a92);
  a92=(a5*a92);
  a87=(a87/a20);
  a87=(a17*a87);
  a87=(a18*a87);
  a87=(a19*a87);
  a92=(a92+a87);
  a87=(a41*a47);
  a87=(a3*a87);
  a87=(a13*a87);
  a92=(a92-a87);
  a88=(a88*a92);
  a84=(a84+a88);
  a88=(a16*a47);
  a88=(a9*a88);
  a88=(a5*a88);
  a47=(a1*a47);
  a47=(a9*a47);
  a47=(a13*a47);
  a88=(a88+a47);
  a89=(a89*a88);
  a84=(a84-a89);
  if (res[1]!=0) res[1][88]=a84;
  a34=(a86*a34);
  a91=(a90*a91);
  a34=(a34+a91);
  a92=(a42*a92);
  a34=(a34+a92);
  a88=(a48*a88);
  a34=(a34-a88);
  if (res[1]!=0) res[1][89]=a34;
  a34=(a5*a42);
  a34=(a3*a34);
  a2=(a2*a34);
  a35=(a35*a86);
  a6=(a6*a35);
  a2=(a2-a6);
  a36=(a36*a86);
  a11=(a11*a36);
  a36=(a5*a48);
  a36=(a9*a36);
  a8=(a8*a36);
  a11=(a11+a8);
  a2=(a2-a11);
  a11=(a13*a48);
  a11=(a9*a11);
  a12=(a12*a11);
  a39=(a39*a86);
  a10=(a10*a39);
  a12=(a12-a10);
  a2=(a2-a12);
  a37=(a37*a86);
  a15=(a15*a37);
  a37=(a13*a42);
  a37=(a3*a37);
  a14=(a14*a37);
  a15=(a15+a14);
  a2=(a2-a15);
  a2=(a86*a2);
  a15=(a19*a42);
  a15=(a18*a15);
  a15=(a17*a15);
  a15=(a15/a23);
  a43=(a43*a90);
  a22=(a22*a43);
  a4=(a4*a22);
  a15=(a15-a4);
  a44=(a44*a90);
  a33=(a33*a44);
  a21=(a21*a90);
  a25=(a25*a21);
  a28=(a28*a25);
  a28=(a28/a29);
  a30=(a30*a25);
  a28=(a28+a30);
  a31=(a31*a28);
  a26=(a26*a31);
  a21=(a21+a21);
  a32=(a32*a21);
  a26=(a26-a32);
  a24=(a24*a26);
  a33=(a33+a24);
  a15=(a15-a33);
  a15=(a90*a15);
  a2=(a2+a15);
  a45=(a45*a86);
  a45=(a3*a45);
  a45=(a5*a45);
  a90=(a90/a20);
  a17=(a17*a90);
  a18=(a18*a17);
  a19=(a19*a18);
  a45=(a45+a19);
  a41=(a41*a86);
  a3=(a3*a41);
  a3=(a13*a3);
  a45=(a45-a3);
  a42=(a42*a45);
  a2=(a2+a42);
  a16=(a16*a86);
  a16=(a9*a16);
  a5=(a5*a16);
  a1=(a1*a86);
  a9=(a9*a1);
  a13=(a13*a9);
  a5=(a5+a13);
  a48=(a48*a5);
  a2=(a2-a48);
  if (res[1]!=0) res[1][90]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int pushing_reduced_expl_ode_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void pushing_reduced_expl_ode_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void pushing_reduced_expl_ode_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int pushing_reduced_expl_ode_hess_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int pushing_reduced_expl_ode_hess_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT const char* pushing_reduced_expl_ode_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pushing_reduced_expl_ode_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pushing_reduced_expl_ode_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pushing_reduced_expl_ode_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int pushing_reduced_expl_ode_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif