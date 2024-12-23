/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__ChR2_william_event
#define _nrn_initial _nrn_initial__ChR2_william_event
#define nrn_cur _nrn_cur__ChR2_william_event
#define _nrn_current _nrn_current__ChR2_william_event
#define nrn_jacob _nrn_jacob__ChR2_william_event
#define nrn_state _nrn_state__ChR2_william_event
#define _net_receive _net_receive__ChR2_william_event 
#define rates rates__ChR2_william_event 
#define states states__ChR2_william_event 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Irradiance _p[0]
#define Irradiance_columnindex 0
#define gmax _p[1]
#define gmax_columnindex 1
#define pulse_width _p[2]
#define pulse_width_columnindex 2
#define light_intensity _p[3]
#define light_intensity_columnindex 3
#define Dt_on _p[4]
#define Dt_on_columnindex 4
#define Dt_off _p[5]
#define Dt_off_columnindex 5
#define nPulses _p[6]
#define nPulses_columnindex 6
#define gamma _p[7]
#define gamma_columnindex 7
#define A _p[8]
#define A_columnindex 8
#define B _p[9]
#define B_columnindex 9
#define C _p[10]
#define C_columnindex 10
#define tauChR2 _p[11]
#define tauChR2_columnindex 11
#define Gd1 _p[12]
#define Gd1_columnindex 12
#define Gd2 _p[13]
#define Gd2_columnindex 13
#define Gr _p[14]
#define Gr_columnindex 14
#define i _p[15]
#define i_columnindex 15
#define O1 _p[16]
#define O1_columnindex 16
#define O2 _p[17]
#define O2_columnindex 17
#define C1 _p[18]
#define C1_columnindex 18
#define p _p[19]
#define p_columnindex 19
#define e12 _p[20]
#define e12_columnindex 20
#define e21 _p[21]
#define e21_columnindex 21
#define epsilon1 _p[22]
#define epsilon1_columnindex 22
#define epsilon2 _p[23]
#define epsilon2_columnindex 23
#define F _p[24]
#define F_columnindex 24
#define S0 _p[25]
#define S0_columnindex 25
#define tally _p[26]
#define tally_columnindex 26
#define DO1 _p[27]
#define DO1_columnindex 27
#define DO2 _p[28]
#define DO2_columnindex 28
#define DC1 _p[29]
#define DC1_columnindex 29
#define Dp _p[30]
#define Dp_columnindex 30
#define v _p[31]
#define v_columnindex 31
#define _g _p[32]
#define _g_columnindex 32
#define _tsav _p[33]
#define _tsav_columnindex 33
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static double _hoc_rates(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "rates", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
#define EChR2 EChR2_ChR2_william_event
 double EChR2 = 0;
#define Q10_epsilon2 Q10_epsilon2_ChR2_william_event
 double Q10_epsilon2 = 2.77;
#define Q10_epsilon1 Q10_epsilon1_ChR2_william_event
 double Q10_epsilon1 = 1.46;
#define Q10_e21dark Q10_e21dark_ChR2_william_event
 double Q10_e21dark = 1.95;
#define Q10_e12dark Q10_e12dark_ChR2_william_event
 double Q10_e12dark = 1.1;
#define Q10_Gr Q10_Gr_ChR2_william_event
 double Q10_Gr = 2.56;
#define Q10_Gd2 Q10_Gd2_ChR2_william_event
 double Q10_Gd2 = 1.77;
#define Q10_Gd1 Q10_Gd1_ChR2_william_event
 double Q10_Gd1 = 1.97;
#define hc hc_ChR2_william_event
 double hc = 1.98645e-25;
#define sigma_retinal sigma_retinal_ChR2_william_event
 double sigma_retinal = 1.2e-19;
#define temp temp_ChR2_william_event
 double temp = 22;
#define wloss wloss_ChR2_william_event
 double wloss = 1.3;
#define wavelength wavelength_ChR2_william_event
 double wavelength = 470;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "Dt_off", 0, 1e+09,
 "Dt_on", 0, 1e+09,
 "nPulses", 0, 1000,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "EChR2_ChR2_william_event", "mV",
 "temp_ChR2_william_event", "degC",
 "gmax", "uS",
 "pulse_width", "ms",
 "Dt_on", "ms",
 "Dt_off", "ms",
 "nPulses", "1",
 "A", "mV",
 "B", "mV",
 "C", "mV",
 "tauChR2", "ms",
 "Gd1", "1./ms",
 "Gd2", "1./ms",
 "Gr", "1./ms",
 "i", "nA",
 0,0
};
 static double C10 = 0;
 static double O20 = 0;
 static double O10 = 0;
 static double delta_t = 0.01;
 static double p0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "EChR2_ChR2_william_event", &EChR2_ChR2_william_event,
 "wavelength_ChR2_william_event", &wavelength_ChR2_william_event,
 "hc_ChR2_william_event", &hc_ChR2_william_event,
 "wloss_ChR2_william_event", &wloss_ChR2_william_event,
 "sigma_retinal_ChR2_william_event", &sigma_retinal_ChR2_william_event,
 "temp_ChR2_william_event", &temp_ChR2_william_event,
 "Q10_Gd1_ChR2_william_event", &Q10_Gd1_ChR2_william_event,
 "Q10_Gd2_ChR2_william_event", &Q10_Gd2_ChR2_william_event,
 "Q10_Gr_ChR2_william_event", &Q10_Gr_ChR2_william_event,
 "Q10_e12dark_ChR2_william_event", &Q10_e12dark_ChR2_william_event,
 "Q10_e21dark_ChR2_william_event", &Q10_e21dark_ChR2_william_event,
 "Q10_epsilon1_ChR2_william_event", &Q10_epsilon1_ChR2_william_event,
 "Q10_epsilon2_ChR2_william_event", &Q10_epsilon2_ChR2_william_event,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"ChR2_william_event",
 "Irradiance",
 "gmax",
 "pulse_width",
 "light_intensity",
 "Dt_on",
 "Dt_off",
 "nPulses",
 "gamma",
 "A",
 "B",
 "C",
 "tauChR2",
 "Gd1",
 "Gd2",
 "Gr",
 0,
 "i",
 0,
 "O1",
 "O2",
 "C1",
 "p",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 34, _prop);
 	/*initialize range parameters*/
 	Irradiance = 0;
 	gmax = 0.4;
 	pulse_width = 100;
 	light_intensity = 5;
 	Dt_on = 100;
 	Dt_off = 50;
 	nPulses = 1;
 	gamma = 0.1;
 	A = 10.6408;
 	B = -14.6408;
 	C = -42.7671;
 	tauChR2 = 1.3;
 	Gd1 = 0.117;
 	Gd2 = 0.05;
 	Gr = 0.00019;
  }
 	_prop->param = _p;
 	_prop->param_size = 34;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[2]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _ChR2H134R_william_event_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 34, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "netsend");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ChR2_william_event /home/gjgpb9/feng_opto_stim/python_bmtk/components/mechanisms/modfiles/ChR2H134R_william_event.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Channelrhodopsin-2 (mutant H134R) current density ";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   rates ( _threadargscomma_ v ) ;
   DO1 = - ( Gd1 + e12 ) * O1 + e21 * O2 + epsilon1 * F * p * C1 ;
   DO2 = - ( Gd2 + e21 ) * O2 + e12 * O1 + epsilon2 * F * p * ( 1. - C1 - O1 - O2 ) ;
   DC1 = - epsilon1 * F * p * C1 + Gd1 * O1 + Gr * ( 1. - C1 - O1 - O2 ) ;
   Dp = ( S0 - p ) / tauChR2 ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 rates ( _threadargscomma_ v ) ;
 DO1 = DO1  / (1. - dt*( ( - ( Gd1 + e12 ) )*( 1.0 ) )) ;
 DO2 = DO2  / (1. - dt*( ( - ( Gd2 + e21 ) )*( 1.0 ) + ( epsilon2 * F * p )*( ( ( - 1.0 ) ) ) )) ;
 DC1 = DC1  / (1. - dt*( ( - epsilon1 * F * p )*( 1.0 ) + ( Gr )*( ( ( - 1.0 ) ) ) )) ;
 Dp = Dp  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauChR2 )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) { {
   rates ( _threadargscomma_ v ) ;
    O1 = O1 + (1. - exp(dt*(( - ( Gd1 + e12 ) )*( 1.0 ))))*(- ( ( e21 )*( O2 ) + ( ( ( epsilon1 )*( F ) )*( p ) )*( C1 ) ) / ( ( - ( Gd1 + e12 ) )*( 1.0 ) ) - O1) ;
    O2 = O2 + (1. - exp(dt*(( - ( Gd2 + e21 ) )*( 1.0 ) + ( epsilon2 * F * p )*( ( ( - 1.0 ) ) ))))*(- ( ( e12 )*( O1 ) + ( ( ( epsilon2 )*( F ) )*( p ) )*( ( 1. - C1 - O1 ) ) ) / ( ( - ( Gd2 + e21 ) )*( 1.0 ) + ( ( ( epsilon2 )*( F ) )*( p ) )*( ( ( - 1.0 ) ) ) ) - O2) ;
    C1 = C1 + (1. - exp(dt*(( - epsilon1 * F * p )*( 1.0 ) + ( Gr )*( ( ( - 1.0 ) ) ))))*(- ( ( Gd1 )*( O1 ) + ( Gr )*( ( 1. - O1 - O2 ) ) ) / ( ( ( ( - epsilon1 )*( F ) )*( p ) )*( 1.0 ) + ( Gr )*( ( ( - 1.0 ) ) ) ) - C1) ;
    p = p + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauChR2)))*(- ( ( ( S0 ) ) / tauChR2 ) / ( ( ( ( - 1.0 ) ) ) / tauChR2 ) - p) ;
   }
  return 0;
}
 
static int  rates ( _threadargsprotocomma_ double _lv ) {
   double _le12dark , _le21dark , _llogphi0 , _lEphoton , _lflux ;
 _le12dark = 0.011 ;
   _le21dark = 0.008 ;
   epsilon1 = 0.8535 ;
   epsilon2 = 0.14 ;
   if ( Irradiance > 0.0 ) {
     _llogphi0 = log ( 1. + Irradiance / 0.024 ) ;
     }
   else {
     _llogphi0 = 0. ;
     }
   e12 = _le12dark + 0.005 * _llogphi0 ;
   e21 = _le21dark + 0.004 * _llogphi0 ;
   S0 = 0.5 * ( 1. + tanh ( 120. * ( 100. * Irradiance - 0.1 ) ) ) ;
   _lEphoton = 1.E9 * hc / wavelength ;
   _lflux = 1000. * Irradiance / _lEphoton ;
   F = _lflux * sigma_retinal / ( wloss * 1000. ) ;
    return 0; }
 
static double _hoc_rates(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _thread = (Datum*)0; _nt = (NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     if ( Irradiance  == 0.0 ) {
       Irradiance = light_intensity ;
       net_send ( _tqitem, _args, _pnt, t +  Dt_on , 1.0 ) ;
       }
     }
   else {
     Irradiance = 0.0 ;
     _lflag = 0.0 ;
     }
   } }
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  C1 = C10;
  O2 = O20;
  O1 = O10;
  p = p0;
 {
   rates ( _threadargscomma_ v ) ;
   C1 = 1. ;
   O1 = 0. ;
   O2 = 0. ;
   p = 0. ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   i = ( gmax * ( A + B * exp ( v / C ) ) / v * ( O1 + gamma * O2 ) * ( v - EChR2 ) ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = O1_columnindex;  _dlist1[0] = DO1_columnindex;
 _slist1[1] = O2_columnindex;  _dlist1[1] = DO2_columnindex;
 _slist1[2] = C1_columnindex;  _dlist1[2] = DC1_columnindex;
 _slist1[3] = p_columnindex;  _dlist1[3] = Dp_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/home/gjgpb9/feng_opto_stim/python_bmtk/components/mechanisms/modfiles/ChR2H134R_william_event.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "-----------------------------------------------------------------------------\n"
  "ChR2H134R.mod\n"
  "\n"
  "Model of Channelrhodopsin-2 (mutant H134R)\n"
  "==========================================\n"
  "\n"
  "from: \n"
  "Williams JC, Xu J, Lu Z, Klimas A, Chen X, et al. (2013) Computational Optogenetics: Empirically-Derived Voltage- and Light-Sensitive Channelrhodopsin- 2 Model. \n"
  "PLoS Comput Biol 9(9): e1003220. doi:10.1371/journal.pcbi.1003220\n"
  "\n"
  "Original (MATLAB) code by: John C. Williams\n"
  "\n"
  "Implemented by: Michele Giugliano, SISSA Trieste, 8/8/2018, mgiugliano@gmail.com\n"
  "\n"
  "This mechanism includes the voltage, light, and temperature dependences of CHR2-H134R. \n"
  "Set the desired temperature by the hoc assignment statement ==> e.g. celsius = 37\n"
  "-----------------------------------------------------------------------------\n"
  "ENDCOMMENT\n"
  "\n"
  "TITLE Channelrhodopsin-2 (mutant H134R) current density \n"
  "\n"
  "\n"
  "UNITS {\n"
  ": Convenient aliases for the units...\n"
  "	(mS) = (millisiemens)\n"
  "	(mV) = (millivolt)\n"
  "	(mA) = (milliamp)\n"
  "}\n"
  "\n"
  "\n"
  "NEURON {\n"
  ": Public interface of the present mechanism...\n"
  "	POINT_PROCESS ChR2_william_event\n"
  "	NONSPECIFIC_CURRENT i\n"
  "\n"
  "	RANGE i, gmax, Irradiance\n"
  "    RANGE light_intensity, light_delay, pulse_width,Dt_on, Dt_off, nPulses,Dt_delay\n"
  "    RANGE Gd1,Gd2,tauChR2,Gr\n"
  "    RANGE A, B, C, gamma\n"
  "\n"
  "    :GLOBAL A, B, C, gamma\n"
  "	GLOBAL wavelength, hc, wloss, sigma_retinal\n"
  "	GLOBAL Q10_Gd1, Q10_Gd2, Q10_Gr, Q10_e12dark, Q10_e21dark, Q10_epsilon1, Q10_epsilon2\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  ": Below are constants, variables that are not changed within\n"
  ": this mechanism, and other variables changed by the user through the hoc code...\n"
  "	Irradiance      = 0.\n"
  "	:gmax  			= 0.4  		(mS/cm2)	  : maximal conductance\n"
  "    gmax  			= 0.4  		(uS)	  : maximal conductance\n"
  "	EChR2 			= 0.     	(mV)          : reversal potential\n"
  "\n"
  "	:light_delay     = 100.		(ms)		  : initial delay, before switching on the light pulse\n"
  "	:Dt_delay     = 100.		(ms)		  : initial delay, before switching on the light pulse\n"
  "    pulse_width     = 100.		(ms)		  : width of the light pulse\n"
  "	light_intensity = 5.					  : mW/mm^2, intensity of the light pulse\n"
  "    \n"
  "    Dt_on     = 100   (ms)    <0, 1e9>        : duration of ON phase  <-Dt_delay->|<-Dt_on->|<-Dt_off->\n"
  "    Dt_off    = 50    (ms)    <0, 1e9>        : duration of OFF phase ________|       |________\n"
  "    nPulses = 1     (1)     <0, 1e3>        : num pulses to deliver         <-- one pulse -->\n"
  "\n"
  "\n"
  "    gamma           = 0.1					  : ratio of conductances in states O2/O1, unit-less\n"
  "	A 				= 10.6408   (mV)          : Be careful with implementing eqs. 1 and 12!\n"
  "	B 				= -14.6408  (mV)		  :\n"
  "	C 				= -42.7671  (mV)          :\n"
  "\n"
  "	wavelength 		= 470		    		  : wavelength of max absorption for retinal, nm\n"
  "	hc       		= 1.986446E-25  		  : Planck's constant * speed of light, kg m^3/s^2\n"
  "	wloss    		= 1.3      : scaling factor for losses of photons due to scattering or absorption\n"
  "	sigma_retinal 	= 12.E-20       		  : retinal cross-sectional area, m^2\n"
  "\n"
  "	tauChR2  		= 1.3		(ms)          : time constant for ChR2 activation\n"
  "\n"
  "	temp 			= 22	  (degC)		  : original temperature\n"
  "	Q10_Gd1      	= 1.97					  : Q10 value for the temperature sensitivity\n"
  "	Q10_Gd2      	= 1.77					  : Q10 value for the temperature sensitivity\n"
  "	Q10_Gr       	= 2.56					  : Q10 value for the temperature sensitivity\n"
  "	Q10_e12dark  	= 1.1					  : Q10 value for the temperature sensitivity\n"
  "	Q10_e21dark  	= 1.95					  : Q10 value for the temperature sensitivity\n"
  "	Q10_epsilon1 	= 1.46					  : Q10 value for the temperature sensitivity\n"
  "	Q10_epsilon2 	= 2.77					  : Q10 value for the temperature sensitivity\n"
  "    \n"
  "    Gd1 = 0.117  (1./ms)   :fixed by Feng\n"
  "    Gd2 = 0.05   (1./ms)  :fixed by Feng\n"
  "    Gr = 1.9E-04 (1./ms)    :fixed by Feng \n"
  "    \n"
  "\n"
  "}\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  ": variables calculated by the present mechanism or by NEURON\n"
  "	v      		(mV)			: Membrane potential\n"
  "	celsius		(degC)          : Temperature\n"
  "\n"
  "	:i    		(mA/cm2)		: Membrane current\n"
  "     i    		(nA)\n"
  "	:Gd1    		(1./ms)			: rate constant for O1->C1 transition\n"
  "	:Gd2    		(1./ms)			: rate constant for O2->C2 transition\n"
  "	:Gr     		(1./ms)			: rate constant for C2->C1 transition\n"
  "	e12    		(1./ms)			: rate constant for O1->O2 transition\n"
  "	e21    		(1./ms)			: rate constant for O2->O1 transition\n"
  "\n"
  "	epsilon1 					: quantum efficiency for photon absorption from C1\n"
  "	epsilon2 					: quantum efficiency for photon absorption from C2\n"
  "	F   		(1./ms)			: photon flux: number of photons per molecule per second\n"
  "	S0							: time- & irradiance-dep. activation func. (post-isomerization)\n"
  "    tally\n"
  "}\n"
  "\n"
  "\n"
  "STATE {\n"
  ": Let's declare the state variables\n"
  ": Note: in any kinetic scheme with N state, you have N-1 *independent* state variables and\n"
  ": this is the reason why C2 is not defined below, but rather expressed as (1 - O1 - O2 - C1). \n"
  ":\n"
  ": O1, O2, C1 and C2 are the fractions of channels in open and closed states, while p is\n"
  ": an additional state variable, capturing the kinetics of ChR2 activation \n"
  "\n"
  "	O1 O2 C1 p\n"
  "}\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE states METHOD cnexp\n"
  "\n"
  "	:Irradiance = 0.               : typically with values in the range 0 - 10 mW/mm^2\n"
  "\n"
  "    : The control of the Irradiance waveform within Neuron is very rudimentary and\n"
  "    : it is made explicit below. In other words, the time course of Irradiance is \n"
  "    : upfront defined - in this example - as a single-pulse photoactivation protocol.\n"
  "    : Modify it to fit your needs!\n"
  "\n"
  "    :if (t < light_delay)                      { Irradiance = 0. }\n"
  "    :else if (t < (light_delay + pulse_width)) { Irradiance = light_intensity }\n"
  "    :else if (t > (light_delay + pulse_width)) { Irradiance = 0. }\n"
  "\n"
  "	i =  (gmax * (A + B * exp(v /C))/v * (O1 + gamma * O2) * (v - EChR2))\n"
  "\n"
  ":printf(\"iopto=%g\\t%g\\t%g\\n\", C1,O1,t)\n"
  "	: note: the above expression includes the voltage-dep. rectification of ChR2\n"
  "}\n"
  "\n"
  "\n"
  "INITIAL {\n"
  ": Let's set the state variables to their initial values... \n"
  "	rates(v)\n"
  "	C1 = 1.\n"
  "	O1 = 0.\n"
  "	O2 = 0.\n"
  "	p  = 0.\n"
  "    \n"
  "    :tally   = nPulses : Set the tally to the number of pulses required \n"
  "    :if (tally > 0) {\n"
  "        :net_send(Dt_delay, 1)\n"
  "        :tally = tally - 1\n"
  "    :}\n"
  "  }\n"
  "\n"
  "\n"
  "DERIVATIVE states {\n"
  ": The state variables are computed here...\n"
  "    rates(v)\n"
  "    : Note: these are the full equations:\n"
  "	:O1' = -(Gd1+e12)           * O1 + e21 * O2 + epsilon1*F*p * C1\n"
  "	:O2' = -(Gd2+e21)           * O2 + e12 * O1 + epsilon2*F*p * C2\n"
  "	:C1' = -epsilon1*F*p        * C1 + Gd1 * O1 + Gr  * C2   \n"
  "	:C2' = -(epsilon2*F*p + Gr) * C2 + Gd2 * O2\n"
  "	:p'  =  (S0 - p) / tauChR2 \n"
  "\n"
  "	: However, only 3 of them are independent. Let's then define C2 as (1. - C1 - O1 - O2)\n"
  "	O1' = -(Gd1+e12)           * O1 + e21 * O2 + epsilon1*F*p * C1\n"
  "	O2' = -(Gd2+e21)           * O2 + e12 * O1 + epsilon2*F*p * (1. - C1 - O1 - O2)\n"
  "	C1' = -epsilon1*F*p        * C1 + Gd1 * O1 + Gr  * (1. - C1 - O1 - O2)  \n"
  "\n"
  "	p'  =  (S0 - p) / tauChR2\n"
  "}\n"
  "\n"
  "\n"
  "UNITSOFF\n"
  "PROCEDURE rates(v (mV)) {\n"
  "    LOCAL e12dark, e21dark, logphi0, Ephoton, flux\n"
  "\n"
  "	: Values at 22 degrees celsius...\n"
  "	e12dark  = 0.011                                 : ms^-1\n"
  "	e21dark  = 0.008                                 : ms^-1\n"
  "	epsilon1 = 0.8535\n"
  "	epsilon2 = 0.14\n"
  "	:Gd1 = 0.075 + 0.043 * tanh( -(v+20.) / 20.)    	 : dark-adapted deactivation rate, ms^-1\n"
  "	:Gd2 = 0.05                                  	 : ms^-1\n"
  "	:Gr  = 0.0000434587 * exp(-0.0211539274 * v)    	 : recovery rate ms^-1\n"
  "    \n"
  "\n"
  "	: These values are adjusted to the temperature specified by the user...\n"
  "	:e12dark  = e12dark  * Q10_e12dark^((celsius-temp)/10.)    : scale with temp, using Q10\n"
  "	:e21dark  = e21dark  * Q10_e21dark^((celsius-temp)/10.)    : scale with temp, using Q10\n"
  "	:epsilon1 = epsilon1 * Q10_epsilon1^((celsius-temp)/10.)   : scale with temp, using Q10\n"
  "	:epsilon2 = epsilon2 * Q10_epsilon2^((celsius-temp)/10.)   : scale with temp, using Q10\n"
  "	:Gd1 	 = Gd1           * Q10_Gd1^((celsius-temp)/10.)	  : scale with temp, using Q10\n"
  "	:Gd2 	 = Gd2           * Q10_Gd2^((celsius-temp)/10.)	  : scale with temp, using Q10\n"
  "	:Gr  	 = Gr             * Q10_Gr^((celsius-temp)/10.)   : scale with temp, using Q10\n"
  "\n"
  "	if (Irradiance>0) {\n"
  "		logphi0  = log(1. + Irradiance / 0.024)      : unit-less\n"
  "	}\n"
  "	else {\n"
  "		logphi0 = 0.	 							 : for consistency\n"
  "	}\n"
  "	e12      = e12dark + 0.005 * logphi0             : ms^-1\n"
  "	e21      = e21dark + 0.004 * logphi0             : ms^-1\n"
  "\n"
  "	S0       = 0.5 * (1. + tanh(120.*(100. * Irradiance - 0.1))) : unit-less\n"
  "	Ephoton  = 1.E9 * hc / wavelength          : J, scaling wavelength from nm to m	\n"
  "	flux     = 1000. * Irradiance / Ephoton    : 1/(s*m^2), scaling irradiance from mW/mm^2 to W/m^2\n"
  "	F        = flux  * sigma_retinal / (wloss * 1000.) : ms^-1, scaling F from 1/s to 1/ms\n"
  "}\n"
  "UNITSON\n"
  "\n"
  "NET_RECEIVE (dummy_weight) {\n"
  "\n"
  "        : flag is an implicit argument of NET_RECEIVE, normally 0\n"
  "    if (flag == 0) { : ignore any but self-events with flag == 1. This may not be necessary... see e.g. nrn/src/nrnoc/intfire1.mod\n"
  "        :phi = phiOn\n"
  "        if (Irradiance==0) {  :only when light was off to on. If it's already on, doing nothing\n"
  "        Irradiance = light_intensity\n"
  "        net_send(Dt_on, 1) : Schedule the next off phase\n"
  "        } \n"
  "        :net_move(t+Cdur_gaba)\n"
  "    } else { : Turn the light off\n"
  "            :printf(\"flag1=%g\\n\", flag)\n"
  "\n"
  "        :phi = 0\n"
  "        Irradiance=0\n"
  "        :if (tally > 0) { : Schedule the next on phase\n"
  "            :net_send(Dt_off, 1)\n"
  "            :tally = tally - 1\n"
  "            flag=0\n"
  "        }\n"
  "            :printf(\"flag=%g\\n\", flag)\n"
  "            :printf(\"t=%g\\n\", t)\n"
  "    }\n"
  ;
#endif
