
// ************** versione  LINUX                *********************
// **************               FINE             *********************
// **************               VER Fin          *********************
// ************** Versione da produrre   1.0.4   *********************
// ************** Versione funzionante   1.0.3.1 *********************
//
//----------------- VERSIONE LINUX -----------------------------------
//
// Questa versione per trasformare i file dxf in codice ISO
//
//--------------------------------------------------------------------


// Taglio linee
// Fori
// Polilinee aperte e chiuse e curvate -non spline-
// Circonferenze
// Archi
// Implementate diverse profondita di passate
// Corretto baco dei dxf con disegni invisibili
// Implementato corretto funzionamento del Joy Stick
// Implementata inversione software rotazione motori
// Corretto bug su fori incompleti
// Corretto problema inversione motori su distanze notevoli (abs torna int)
// Implementata ottimizzazione percorso utensile  <------- Verificare in uso! Non funziona sempre rivedere !!!
// Implementato microtiming (Ver. 5005)
// Implementata rampa su movimenti manuali (Ver. 5006)
// Implementata rampa su spostamenti rapidi (Ver. 5006)
// Implementate V min partenza e Vmax su spostamenti rapidi (Ver. 5007)
// Implementate Vmin e Vmax di taglio (Ver. 5007)
// Eliminato baco su multipassata: polilinee con archi scorrette dopo prima passata (Ver. 5008)
// Implementata simulazione grafica rapida (5009)
// Verificati step inviati ma non perkŠ incongruenza nel reale(5009)
// Verifica con TurboCNC, stessi errori se lento, maggiori se VT aumenta
// Implementata corsa rapida su abbassamento utensile
// Implementata rampa su taglio (5011)
// Implementata lettura coordinate in manuale e azzeramento(5011)
// Versione 5011 che funzionicchia ma c'Š problema di memoria allocata da riordino
// Risolto assegnando area fissa ai nomi dei file con array di char
// da ancora errore di floating point a volte (5012)

// la coordinata Z sarà trattata come da coordinata fornita dall'entità oppure, per entità 2D dalla profondità
// di passata di default, queste verranno poi dedotte o da testo e/o da layer. Lo stesso per la velocità
// di taglio


// questa versione 0.0 derivante dalla 5012_3 per Linux
// supporta la modalità grafica
// supporta le LWPOLYLINE
// sembra funzionare bene al 30/08/06// scompaiono dei cerchi .... fumo.dxf(mancano 2 cerchi)  e fumo_exp.dxf
// risolto passando np come parametro... applicare anche alle altre entita
// implementato il passaggio dei parametri da riga di comando

// versione 12 ripulita da variabili non usate
// versione 13 rimosse routine grafiche
// versione 14 rimossi menu e file dati
// versione 15 rimuovere i parametri e le dipendenze hardware da file default.dat
// versione 16 rimosse le pause
// versione 17 correggere la differenze tra i raggi segnalate da jepler su #emc..... fatto!
// versione 17 rimuovere le dipendenze hardware
// versione 1.0.3 aggiunto output su standard output per fornire input a Emc2
// versione 1.0.3.1 corretto baco su polilinee 2D senza Z da Ray Abram
// versione 1.0.4
//			aggiunto il modo G61, G61.1 e G64
//			http://www.linuxcnc.org/docview/html//gcode_main.html#sub:G61,-G61.1,-G64:
//			Aggiunta la possibilità di passare i parametri di taglio delle entità tramite nome del layer
// 28/04/2025
// 		Risolto problema della lwpolyline aperta che quando tagliata in più passate, non alza la punta 
// 		per tornare al punto di partanza per passate successive

#define _GNU_SOURCE  // WEB added since fcloseall would otherwise not be defined

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

//#include <sys/io.h>  //per outb
#include <stdbool.h>

#include <sys/select.h>

#include <math.h>

#include <stdlib.h>

#include <stdio.h>

#include <string.h>

void taglia(void);
void sposta_punta(float ddx, float ddy, float ddz, int speed, int modot);
float abso_float(float value);
long abso_long(long value);
#ifdef NEED_ROUND  // WEB took out function round to avoid redefinition
long  round(float f);

#endif  // WEB endif
void cut_polyline(int colore, int volte); //interno 1 esterno 0
void cut_lwpolyline(int colore, int volte);
void cut_circle(int colore, int volte);
void cut_point(int colore, float attesa, int volte);
void cut_arco(float dx, float dy, float bulge);
void cut_arc(int colore, int volte);
void cut_line(int colore, int volte);

void print_screen(void);
void stampa_help(void);
void evaluate_layer_name(void);

struct entita{
	unsigned int N;		// posizione nel file originale dxf
	unsigned int tipo;	// tipo di entita
	unsigned int ie;	// interno 0 esterno 7
				// 1-> polilinea
				// 2-> linea
				// 3-> punto
				// 4-> arco
				// 5-> cerchio
				// 6-> lwpolyline
	float xi;		// punto di partenza -- per CIRCLE centro
	float yi;
	float zi;		// aggiunto per versione 1.0.4
	float xf;		// punto arrivo -- per CIRCL reggio
	float yf;
	float zf;		// aggiunto per versione 1.0.4
	float feed;		// aggiunto per versione 1.0.4
	float prof;		// aggiunto per versione 1.0.4
	unsigned int ndp;	// aggiunto per versione 1.0.4
};

void scambia(struct entita  *ent1, struct entita *ent2);
float distanza(struct entita *ent_par, struct entita *ent_arr);
void crea_file_ordinato(void);

void zero_assi(void);


char ver[]="1.0.6";
char data[]="09/04/2025";


int cerchi_cut=0, fakeint;

float pvx, pvy, pvz, pp, dp, dit;
int smx, smy, smz, delmin, vt, np, np_orig, xv;
int errore=0, err_ch, simula;

char nome_file[255];

char cut_file[8]="cut.cut";    // nome del file di taglio
char cnc_file[255];

char strinp[255];
char stringa[255];

char nome_file_comp[255];

/* modalita ---> The machining center may be put into any one of three path control modes: (1) exact stop mode, (2) exact path mode, or (3) continuous mode with optional tolerance. In exact stop mode, the machine stops briefly at the end of each programmed move. In exact path mode, the machine follows the programmed path as exactly as possible, slowing or stopping if necessary at sharp corners of the path. In continuous mode, sharp corners of the path may be rounded slightly so that the feed rate may be kept up (but by no more than the tolerance, if specified).
Program G61 to put the machining center into exact path mode, G61.1 for exact stop mode, or G64 P- for continuous mode with optional tolerance. G61 visits the programmed point exactly, even though that means temporarily coming to a complete stop. G64 without P means to keep the best speed possible, no matter how far away from the programmed point you end up.*/
char modalita[11];

// nome del layer nel formato:
// DPxxxx.xxxxNPxxxFxxxx.xxxx
// DP prof di passata NP numero delle passate di profondità DP, F feed
char layer_name[26];
char dp_float_entity[9], f_float_entity[9], np_int_entity[3];
float dp_entity, f_entity;
unsigned int np_entity;
char dp_layer_name_code[2];
char np_layer_name_code[2];
char f_layer_name_code[1];
int ent_con_par_propri = 0;

char lang[255]; //nome file messaggi a video .\lang\xxxxxxxx.xxx

char dirdis[255]; // nome sottodir disegni .\xxxxxxxx
char unita[2]; // lettera m o i + \0

FILE *input_file, *output_file_cnc;

FILE *log_file;
char cut_log_file[8]="cut.log"; 

// vt velocita di taglio, np numero di passate, pp profondita di passata
//dp distanza z di partenza della punta dal pezzo
float x=0, y=0, z=0, xerr=0, yerr=0, zerr=0, xr=0, yr=0, zr=0, posx=0, posy=0, posz=0;
float resx, resy, resz, scalax, scalay;
double pi;
// x,y,zerr errore di posizione, x,y,z, posizione teorica, xyzr reale multipla degli step fatti dai motori
// resx resy resz risoluzione pv/sm
int yv; // dimensioni della finestra video

int maxx, maxy; //dimensioni finestra driver grafico
int oex, oey, oez, oen;
			       // switch abilitazione motori
int dir_mx, dir_my, dir_mz; //switch direzione motori
int lpt_nmbr; //numero porta parallela da usare

int  ent_in_esame=0;

int x0_vp, y0_vp, xdraw_old, ydraw_old;

unsigned int *pAddr;

unsigned int DATA_addr, STATUS_addr, CONTROL_addr;

unsigned int pin_step_mx, pin_step_my, pin_step_mz;
unsigned int pin_dir_mx, pin_dir_my, pin_dir_mz;
unsigned int pin_oen_mx, pin_oen_my, pin_oen_mz;
unsigned int res_arc, ticks, vre;
unsigned long dtmin, dtmax;// Vmax rapida e v partenza rampa
unsigned long ramp_step;
unsigned long double_ramp, deltick; // parametri rampa
unsigned long delta_t_on; //varazione t_on in taglio in aumento o dim VT
unsigned long delmin_cut, delmax_cut; //parametri VTmax e VTmin
long stepx_f, stepy_f, stepz_f; // controllo
long sxfd, syfd, szfd;
float dwell=0; //attesa per la routine di foratura
float rt=0;// rompitruciolo
// ************************************************************************

int main (int argc, char *argv[]){
	
	int opt;
	
	pi=acos(-1);
	pvx=pvy=pvz=4;
	smx=smy=smz=400;
	delmin_cut=25000;
	vt=5000;
	np=1;
	pp=1.0;
	dp=6.0;
	lpt_nmbr=1;
	pin_step_mx=2;
	pin_dir_mx=3;
	pin_step_my=6;
	pin_dir_my=7;
	pin_step_mz=4;
	pin_dir_mx=5;
	pin_oen_mx=16;
	pin_oen_my=1;
	pin_oen_mz=14;
	oen=1;
	ticks=60000;
	dtmin=20000;
	res_arc=10;
	dir_mx=1;
	dir_my=1;
	dir_mz=0;
	dtmax=300000;
	deltick=1800;
	vre=90;
	dit=2;
	ramp_step=(dtmax-dtmin)/deltick;  //step di rampa
	double_ramp=ramp_step*2;
	// parametri taglio
	delta_t_on=(dtmax-dtmin)/100;
	*nome_file_comp='\0';  // WEB fixed missing cast warning
	switch(lpt_nmbr){ //recupero indirizzo lpt usata
		case 1:
			// /dev/lp0
			DATA_addr=0x3BC;
			STATUS_addr=DATA_addr+1;
			CONTROL_addr=DATA_addr+2;
		case 2:
			// /dev/lp1
			DATA_addr=0x378;
			STATUS_addr=DATA_addr+1;
			CONTROL_addr=DATA_addr+2;
		case 3:
			// /dev/lp2
			DATA_addr=0x278;
			STATUS_addr=DATA_addr+1;
			CONTROL_addr=DATA_addr+2;
		default:
			//pAddr=(unsigned int *) 0x0408;
			DATA_addr=0x3BC;
			STATUS_addr=DATA_addr+1;
			CONTROL_addr=DATA_addr+2;
	}
	pin_step_mx=pow(2,(pin_step_mx-2));
	pin_step_my=pow(2,(pin_step_my-2));
	pin_step_mz=pow(2,(pin_step_mz-2));
	pin_dir_mx=pow(2,(pin_dir_mx-2));
	pin_dir_my=pow(2,(pin_dir_my-2));
	pin_dir_mz=pow(2,(pin_dir_mz-2));
	switch(pin_oen_mx){
		case 1:
			pin_oen_mx=1;
			break;
		case 14:
			pin_oen_mx=2;
			break;
		case 16:
			pin_oen_mx=4;
			break;
		case 17:
			pin_oen_mx=8;
			break;
	}
	switch(pin_oen_my){
		case 1:
			pin_oen_my=1;
			break;
		case 14:
			pin_oen_my=2;
			break;
		case 16:
			pin_oen_my=4;
			break;
		case 17:
			pin_oen_my=8;
			break;
	}
	switch(pin_oen_mz){
		case 1:
			pin_oen_mz=1;
			break;
		case 14:
			pin_oen_mz=2;
			break;
		case 16:
			pin_oen_mz=4;
			break;
		case 17:
			pin_oen_mz=8;
			break;
	}
	
	zr=dp;
	resx=(float)pvx/(float)smx;
	resy=(float)pvy/(float)smy;
	resz=(float)pvz/(float)smz;
	
	printf("\n\n\n(DXF2G GPROGRAMMA PER LA CONVERSIONE DI FILE DAL FORMATO DXF A CODICE G)\n");
	printf("(AUTORE: Ing. Pierpaolo Garofalo)\n");
	printf("(Versione: %s)\n",ver);
	printf("(Data: %s)\n\n\n",data);
	printf("dxf2G  Copyright (C) 2010  Pierpaolo Garofalo\n");
	printf("This program comes with ABSOLUTELY NO WARRANTY\n");
	printf("This is free software, and you are welcome to redistribute it\n");
	printf("under certain conditions;\n\n\n");
#define DEBUG
#ifdef DEBUG
	printf("\n#Compilato il: " __DATE__ " alle: " __TIME__ "\n");
#endif
	
	zr=dp;
// n = numero di passate -------------> np
// p = profondità delle passate ------> pp
// d = distanza punta pezzo ----------> dp
// v = velocità di taglio ------------> vt
// u = mm o inch ---------------------> unita
// x = dimensione finestra video -----> xv
// f = file.dxf ----------------------> nome_file
// i = distanza inizio taglio --------> dit
// e = modo esatto G61 ---------------> em
// inizializzazione dei valori di default
	np=vt=1;
	xv=200;
	pp=dp=dit=1.0;
	//

	// strncpy copia la stringa src in dest per n byte. se src è più corta di dest riempie la coda con \0.
	// Se src è più lunga di dest. per evitare l'overflow, copia solo i primi n-1 spazi di dest e nell'ultimo metto '\0'
	strncpy(unita,"m", sizeof(unita)-1);
	unita[sizeof(unita)-1] = '\0';
	strncpy(modalita,"G61", sizeof("G61")-1);
	modalita[sizeof(modalita)-1] = '\0';
	while((opt=getopt(argc, argv, "n:p:d:v:u:f:i:e:h")) !=-1){
		switch(opt){
			case 'n':
				np=atoi(optarg);
				printf("(Default number of passes = %d)\n", np);
				break;
			case 'p':
				pp=atof(optarg);
				printf("(Default cutting depth = %lf)\n", pp);
				break;
			case 'd':
				dp=atof(optarg);
				printf("(Tool to piece distance = %lf)\n", dp);
				break;
			case 'v':
				vt=atoi(optarg);
				printf("(Default feed = %d)\n", vt);
				break;
			case 'u':
				// m -> metrica G21 i -> anglo G20
				strncpy(unita,optarg,sizeof(unita)-1);//strlen(optarg));
				unita[sizeof(unita)-1] = '\0';
				printf("(Units = %s)\n", unita);
				break;
			case 'f':
				if(strlen(optarg)<255){
					strncpy(nome_file_comp,optarg,sizeof(nome_file_comp)-1);
					nome_file_comp[sizeof(nome_file_comp)-1] = '\0';
					printf("(File name = %s)\n", nome_file_comp);
				}
				else
					printf("\nFile name too long\n");
				break;
			case 'i':
				dit=atof(optarg);
				printf("(Cutting distance = %lf)\n", dit);
				break;
			case 'e':
				if(strlen(optarg)<12){
					strncpy(modalita,optarg,sizeof(modalita)-1);
					modalita[sizeof(modalita)-1] = '\0';
					printf("(Path mode = %s)\n", modalita);
				}
				else{
					printf("\nPath mode provided not supported\n");
					exit(1);
				}
				break;
			case 'h':
				stampa_help();
				exit(0);
				break;
			case '?':
				printf("\n\n%s: Unknown option! Type dxf2G -h to get help or consult manual page.\n\n", optarg);
				stampa_help();
				exit(0);
		}
	}
	if (strcmp(nome_file_comp,"")==0){
		printf("\nFilename must be provided! \n");
		stampa_help();
		exit(0);
	}
	input_file=fopen(nome_file_comp,"r");
	if(input_file==NULL){
		printf("\nDrawing file does not exist!\n");
		exit(0);
	}
	else{
		printf("\nSorting cnc path\n");
		crea_file_ordinato();
		printf("Done!\n\n");
	}
	fclose(input_file);
	taglia();
	//print_screen();
	return(0);
}
// ******************************** fine main ***************************
// ***********************************************************************
void crea_file_ordinato(void){      // inizio funzione
	//
	float xc, yc, r, alfapart, alfafine, dist_min;
	struct entita *entita_pr, *org_entita_pr, *temp_entita_pr;
	// definizione del puntatore a entita
	//struct entita entita_swap;
	// entita di parcheggio per lo swap di riordino
	struct entita origine;		//punto di partenza utensile
	FILE *cut_file_pr;			// puntatore al file di taglio
	unsigned int i, j, trovato, num_entita=0, num_ent_int, num_ent_est,  color_ent;
	unsigned int n_cerchi=0, n_poliline=0, n_linee=0, n_punti=0, n_archi=0, n_lwpoly=0;
	unsigned int posizione=0, aperta, nvp=0, ok=0;	//numero vertici polilinea
	float xd, yd, zd, bulge_next;
	
	origine.N   =0; // non serve
	origine.tipo=3; // punto
	origine.ie  =0; // non serve ie=interna/esterna
	origine.xi  =0;
	origine.yi  =0;
	origine.xf  =0;
	origine.yf  =0;
	//************** conteggio entita da tagliare
	for(trovato=0;trovato<1;){
		fakeint = fscanf(input_file,"%s",strinp);
		if(strncmp(strinp,"ENTITIES",8)==0)
			trovato=1;
	} //dovrei evitare le porcherie invisibili!!!!!!!!!!!!
	for(;;){  // inizio for 1
		fakeint = fscanf(input_file,"%s",strinp);
		// vedo se il comando e' una poliline
		if(strncmp(strinp,"POLYLINE",8)==0){
			num_entita++;
			n_poliline++;
		}
		if(strncmp(strinp,"CIRCLE",6)==0){
			num_entita++;
			n_cerchi++;
		}
		if(strncmp(strinp,"POINT",5)==0){
			num_entita++;
			n_punti++;
		}
		if(strncmp(strinp,"ARC",3)==0){
			num_entita++;
			n_archi++;
		}
		if(strncmp(strinp,"LINE",4)==0){
			num_entita++;
			n_linee++;
		}
		if(strncmp(strinp,"LWPOLYLINE",10)==0){
			n_lwpoly++;
			num_entita++;
		}
		// se non sono alla fine del file elaboro un'altra linea senno riavvolgo
		if(strncmp(strinp,"EOF",3)==0){
			rewind(input_file);
			break;
		}
	}    // fine for 1
	
	printf("(%d \tentities found!)\n",num_entita);
	printf("(%d \tcircles found!)\n",n_cerchi);
	printf("(%d \tarcs found!)\n",n_archi);
	printf("(%d \tpoints found!)\n",n_punti);
	printf("(%d \tlines found!)\n",n_linee);
	printf("(%d \tpolylines found!)\n",n_poliline);
	printf("(%d \tlwpolylines found!)\n",n_lwpoly);
	// alloco memoria per struttura delle entita da riordinare
	if ((entita_pr = calloc(num_entita,sizeof(struct entita))) == NULL){
		printf("\n\nNot enough memory to allocate buffer");
		fclose(input_file);
		exit(1);  /* terminate program if out of memory */
	}
	else{
		org_entita_pr=entita_pr; // salvo l'indirizzo iniziale di memoria
	}
	// rilettura del file dxf per memorizzare le entita in memoria
	for(trovato=0;trovato<1;){ // mi rimetto all'inizio delle entities
		fakeint = fscanf(input_file,"%s",strinp);
		if(strncmp(strinp,"ENTITIES",8)==0)
			trovato=1;
	}
	//dovrei evitare le porcherie invisibili!!!!!!!!!!!!
	// recupero dati entita 
	for(;;){ //inizio for 2  per recuperare dati entita
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------------- inizio polyline
		if(strncmp(strinp,"POLYLINE",8)==0){
			aperta=1;
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=1;
			nvp=0;
			for(;;){	// inizio for scansione polilinea
				fakeint = fscanf(input_file,"%s",strinp);
				// vedo se la poli e' chiusa o aperta valutando il codice
				// 70 prima dei VERTEX
				if((strncmp(strinp,"70",2)==0)&(nvp==0)){
					// solo fuori da vertex ossia nvp = 0
					fakeint = fscanf(input_file,"%s",strinp);
					//if((strncmp(strinp,"1",1)==0)|(strncmp(strinp,"3",1)==0))
					aperta = (atoi(strinp) & 1)? 0 : 1;
						//aperta=0;
					// allora la polilinea e' chiusa
					// dovro' aggiungere un vertice coinc
					// col primo nel file di taglio temporaneo
				}
				// vedo se ho trovato un vertice
				if(strncmp(strinp,"VERTEX",6)==0){
					// si quindi incremento il numero di vertici nvp
					nvp++;
					for(;;){
						// inizio for scansione interna vertice
						fakeint = fscanf(input_file,"%s",strinp);
						// valuto se il codice letto è 10 cioè x del Vertex
						if(strcmp(strinp,"10")==0){
							// se si allora seguono 20 e 30 cioè xy e z
							fakeint = fscanf(input_file,"%f",&xd);
							fakeint = fscanf(input_file,"%s",strinp); // 20
							fakeint = fscanf(input_file,"%f",&yd);    // Y
							fakeint = fscanf(input_file,"%s",strinp); // 30?
							//====================== fixing here by Ray Abram ==========
							if (strncmp(strinp,"30",2)==0){
								fakeint = fscanf(input_file,"%f",&zd);    // Z
								fakeint = fscanf(input_file,"%s",strinp); // Read next command      42?
							}
							else{
								zd = 0;
							}
							//===========================================================
							//vedo se il codice che segue la eventuale Z vale 42 ossia raccordo
							if(strncmp(strinp,"42",2)==0)
								// se si allora leggo la tang dell'angolo/4 dell'arco
								// con cui raggiungere il verice che segue
								fakeint = fscanf(input_file,"%f",&bulge_next);  // WEB changed to fakeint = fscanf to fix warning about first parameter ???
							else
								// altrimenti non c'Š raccordo
								bulge_next=0;
							// scrivo i dati del vertice nel file temporaneo di taglio
							if(nvp==1){
								entita_pr->xi=xd;
								entita_pr->yi=yd;
							}
							break; // esco dal for di scansione interno al vertice
						}
						else if (strncmp(strinp,"EOF",3)==0){
							printf("\nFile anomalo!");
							err_ch=fcloseall();
							return;
						}
					}   // fine for scansione interna vertice
				}
				// vedo se sono finiti i vertici
				if(strncmp(strinp,"SEQEND",6)==0){
					// inizio if SEQEND
					// giunto a fine polilinea
					// vedo se la polilinea Š chiusa
					if(aperta==0){
						 // se la polilinea e' chiusa
						// il punto di partenza e finale coincidono
						entita_pr->xf=entita_pr->xi;
						entita_pr->yf=entita_pr->yi;
					}
					else{
						// altrimenti il punto finale Š l'ultimo letto
						entita_pr->xf=xd;
						entita_pr->yf=yd;
					}
					break; // finita scansione polilinea. esco dal for scansione polilinea
				} // fine if SEQEND
				if(strcmp(strinp,"62")==0){
					// verifico il codice colore della polilinea per decidere
					// se Š interna 0 o esterna 7
					fakeint = fscanf(input_file,"%ud",&color_ent);
					if(color_ent==7)
						entita_pr->ie=7;
					else
						entita_pr->ie=0;
				}
			}
			// fine for scansione polilinea
			entita_pr++;
		}
		// --------------------------- fine polyline
		//--------------------------- inizio lwpolyline
		if(strncmp(strinp,"LWPOLYLINE",10)==0){
			aperta=1;
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=6;
			ok=0;
			nvp=0;
			for(;;){
				// inizio for scansione lwpolilinea
				// il codice che segue 70 è un bitmask
				// 
				fakeint = fscanf(input_file,"%s",strinp);
				// vedo se la poli e' chiusa o aperta valutando il codice 70
				// se il codice è dispari è chiusa, altrimenti è aperta
				if(strncmp(strinp,"70",2)==0){
					// in LWpolyline non ci sono i Vertex
					fakeint = fscanf(input_file,"%s",strinp);
					//if((strncmp(strinp,"1",1)==0)|(strncmp(strinp,"3",1)==0))
					aperta = (atoi(strinp) & 1)? 0 : 1;
					// allora la polilinea e' chiusa dovro' aggiungere un vertice coinc
					// col primo nel file di taglio temporaneo
					ok=1;
				}
				if(strncmp(strinp,"90",2)==0){
					// in lwpolyline 90 è il codice del numero di vertici
					fakeint = fscanf(input_file,"%ud",&nvp);
					ok=1;
				}
				// vedo se ho trovato un vertice
				if((strcmp(strinp,"10")==0)&(nvp!=0)){
					// valuto se il codice letto è 10 cioè x
					for(i=0;i<nvp;i++){
						fakeint = fscanf(input_file,"%f",&xd);
						fakeint = fscanf(input_file,"%s",strinp); // 20
						fakeint = fscanf(input_file,"%f",&yd);    // Y
						fakeint = fscanf(input_file,"%s",strinp); // 42?
						//vedo se il codice che segue la Z vale 42 ossia raccordo
						if(strncmp(strinp,"42",2)==0)
							// se si allora leggo la tang dell'angolo/4 dell'arco
							// con cui raggiungere il verice che segue
							fakeint = fscanf(input_file,"%f",&bulge_next);
						else
							// altrimenti non c'Š raccordo
							bulge_next=0;
						// scrivo i dati del vertice nel file temporaneo di taglio
						if(i==0){
							entita_pr->xi=xd;
							entita_pr->yi=yd;
						}
					}
					// giunto a fine polilinea vedo se la polilinea è chiusa
					if(aperta==0){
						// se la polilinea e' chiusa il punto di partenza e finale coincidono
						entita_pr->xf=entita_pr->xi;
						entita_pr->yf=entita_pr->yi;
					}
					else{
						// altrimenti il punto finale Š l'ultimo letto
						entita_pr->xf=xd;
						entita_pr->yf=yd;
					}
					break;
					// finita scansione polilinea. esco dal for scansione polilinea
				}
				if(strcmp(strinp,"62")==0){
					// verifico il codice colore della polilinea per decidere
					// se è interna 0 o esterna 7
					fakeint = fscanf(input_file,"%ud",&color_ent);
					ok=1;
					if(color_ent==7)
						entita_pr->ie=7;
					else
						entita_pr->ie=0;
				}
				if(ok==0)
					// il codice letto non coorisp a nessuno di quelli contemplati
					// devo quindi leggere comunque il valore ad esso corrispondente
					fakeint = fscanf(input_file,"%s",strinp);
			}
			// fine for scansione polilinea
			entita_pr++;
		}
		//------------------------------------------------ fine lwpolyline
		//------------------------------------------------ inizio circle
		if(strncmp(strinp,"CIRCLE",6)==0){
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=5;
			for(;;){
				// inizio for intero a CIRCLE per raccolta dati
				fakeint = fscanf(input_file,"%s",strinp);
				if(strcmp(strinp,"10")==0){
					fakeint = fscanf(input_file,"%f",&entita_pr->xi);		// x del centro
					fakeint = fscanf(input_file,"%s",strinp);				// 20
					fakeint = fscanf(input_file,"%f",&entita_pr->yi);		// y del centro
					fakeint = fscanf(input_file,"%s",strinp);				// 30
					fakeint = fscanf(input_file,"%s",strinp);				// z del centro
					fakeint = fscanf(input_file,"%s",strinp);				// 40      non serve
					fakeint = fscanf(input_file,"%f",&entita_pr->xf);		// R del cerchio
					entita_pr->yf=0;							// yf con serve per il cerchio
					break;										// fine raccolta dati cerchio esco dal for
				}
				if(strcmp(strinp,"62")==0){
					// verifico il codice colore della polilinea per decidere
					// se è interna 0 o esterna 7
					// inizio if colore cerchio
					fakeint = fscanf(input_file,"%ud",&color_ent);
					if(color_ent==7)
						entita_pr->ie=7;
					else
						entita_pr->ie=0;
				}
				// fine if colore cerchio
			}
			// fine for intero a CIRCLE per raccolta dati
			entita_pr++;
		}
		//------------------------------------------------ fine circle
		//------------------------------------------------ inizio point
		if(strncmp(strinp,"POINT",5)==0){
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=3;
			for(;;){
				// inizio for scansione interna POINT
				fakeint = fscanf(input_file,"%s",strinp);
				if(strcmp(strinp,"10")==0){
					// inizio if recupero coordinate point
					fakeint = fscanf(input_file,"%f",&entita_pr->xi);	// X del punto
					fakeint = fscanf(input_file,"%s",strinp);			// 20
					fakeint = fscanf(input_file,"%f",&entita_pr->yi);	// Y del punto
					fakeint = fscanf(input_file,"%s",strinp);			// 30
					fakeint = fscanf(input_file,"%s",strinp);			// Z non serve
					entita_pr->xf=0;
					entita_pr->yf=0;
					break;}
					// fine if recupero coordinate point
					if(strcmp(strinp,"62")==0){
						// inizio if colore POINT
						fakeint = fscanf(input_file,"%ud",&color_ent);
						if(color_ent==7)
							entita_pr->ie=7;
						else
							entita_pr->ie=0;
					}
					// fine if colore POINT
			}
			// fine for scansione interna POINT
			entita_pr++;
		}
		//------------------------------------------------ fine point
		//------------------------------------------------ inizio arc
		if(strncmp(strinp,"ARC",3)==0){
			// inizio if ARC
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=4;
			for(;;){
				// inizio for scansione ARC
				fakeint = fscanf(input_file,"%s",strinp);
				if(strcmp(strinp,"10")==0){
					// inizio if recupero coordinate
					fakeint = fscanf(input_file,"%f",&xc);		// x centro arco
					fakeint = fscanf(input_file,"%s",strinp);		// 20
					fakeint = fscanf(input_file,"%f",&yc);		// Y centro arco
					fakeint = fscanf(input_file,"%s",strinp);		// 30
					fakeint = fscanf(input_file,"%s",strinp);		// z centro arco non serve
					fakeint = fscanf(input_file,"%s",strinp);		// 40
					fakeint = fscanf(input_file,"%f",&r);			// R arco
					fakeint = fscanf(input_file,"%s",strinp);		// 50
					fakeint = fscanf(input_file,"%f",&alfapart); 	// angolo partenza in gradi !!!
					fakeint = fscanf(input_file,"%s",strinp);		// 51
					fakeint = fscanf(input_file,"%f",&alfafine);	// angolo arrivo in gradi !!!
					break;
				}
				// fine if recupero coordinate
				if(strcmp(strinp,"62")==0){
					// inizio if gestione colore
					fakeint = fscanf(input_file,"%ud",&color_ent);
					if(color_ent==7)
						entita_pr->ie=7;
					else
						entita_pr->ie=0;
				}
				// fine if gestione colore
			}
			 // fine for scansione ARC
			// calcolo punti partenza e arrivo
			alfapart=alfapart*pi/180;			// in radianti
			alfafine=alfafine*pi/180;			// in radianti
			entita_pr->xi=xc+r*cos(alfapart);	// x punto partenza
			entita_pr->yi=yc+r*sin(alfapart);	// y punto partenza
			entita_pr->xf=xc+r*cos(alfafine);	// x punto arrivo
			entita_pr->yf=yc+r*sin(alfafine);	// y punto arrivo
			entita_pr++;
		}
		//------------------------------------------------ fine arc
		//------------------------------------------------ inizio line
		if(strncmp(strinp,"LINE",4)==0){
			posizione++;
			entita_pr->N=posizione;
			entita_pr->tipo=2;
			for(;;){
				// inizio for scansione interna LINE
				fakeint = fscanf(input_file,"%s",strinp);
				if(strcmp(strinp,"10")==0){
					// inizio if recupero coordinate
					fakeint = fscanf(input_file,"%f",&entita_pr->xi);	// x punto iniziale
					fakeint = fscanf(input_file,"%s",strinp);			// 20
					fakeint = fscanf(input_file,"%f",&entita_pr->yi);	// y punto iniziale
					fakeint = fscanf(input_file,"%s",strinp);			// 30
					fakeint = fscanf(input_file,"%s",strinp);			// z non serve
					fakeint = fscanf(input_file,"%s",strinp);			// 11
					fakeint = fscanf(input_file,"%f",&entita_pr->xf);	// x punto finale
					fakeint = fscanf(input_file,"%s",strinp);			// 21
					fakeint = fscanf(input_file,"%f",&entita_pr->yf);	// y punto finale
					fakeint = fscanf(input_file,"%s",strinp);			// 31
					fakeint = fscanf(input_file,"%s",strinp);			// z non serve
					break;}
					// fine recupero coordinate
					if(strcmp(strinp,"62")==0){
						// inizio if colore
						fakeint = fscanf(input_file,"%ud",&color_ent);
						if(color_ent==7)
							entita_pr->ie=7;
						else
							entita_pr->ie=0;
					}
					// fine if colore
			}
			// fine for scansione interna LINE
			entita_pr++;
		}
		//------------------------------------------------ fine line
		if(strncmp(strinp,"EOF",3)==0){
			break;
			// giunto alla fine del file quindi esco dal for 2
		}
	}
	//fine for 2 per recuperare dati entita
	// Ora devo riordinare la matrice tenendo conto che bisogna
	// tagliare prima le parti interne
	entita_pr=org_entita_pr;
	// ripunto all'origine della matrice
	num_ent_int=0;
	// conto le entita interne quelle con ie=0
	for(i=0;i<num_entita;i++){
		// inizio for conto entita interne
		if(entita_pr->ie==0)
			num_ent_int++;
		entita_pr++;
	}
	entita_pr=org_entita_pr;
	// ora porto le entita interne da tagliare all'inizio se esistono
	temp_entita_pr=org_entita_pr;
	if(num_ent_int>0){
		for(i=0;i<num_entita;i++){
			// inizio for raggruppamento ent interne
			if(entita_pr->ie==0){
				// incontrata entita interna di taglio faccio lo swap
				scambia(entita_pr,temp_entita_pr);
				temp_entita_pr++;
				 // punta alla prima esterna
			}
			entita_pr++;
			// spazza tutte le entita
		}
		// fine  for raggruppamento ent interne
	}
	// da adesso se esiste almeno una entita int allora è al primo posto
	entita_pr=org_entita_pr;
	temp_entita_pr=org_entita_pr;
	// riporto a zero i puntatori
	// adesso riordino le entita col criterio della distanza
	// partendo da quelle interne
	entita_pr=org_entita_pr;
	temp_entita_pr=entita_pr;
	dist_min=distanza(&origine,entita_pr);
	//assumo arbitrariamente d_min
	// quella tra O e il primo elemento
	// sia esso int che est 
	// se num_ent_int >1 allora entro nel for per ricerca ent int + vicina a O
	// se num_ent_int=1 allora Š gia al primo posto
	if(num_ent_int>1){
		for(i=0;i<num_ent_int;i++){
			// viene eseguito se ho + di 1 ent_int
			// metto al primo posto l'entita + vicina all'origine degli assi
			if(distanza(&origine,entita_pr)<dist_min){
				scambia(org_entita_pr,entita_pr);
				dist_min=distanza(&origine, org_entita_pr);
			}
			entita_pr++;
		}
		// ora l'entita interna + vicina a O è al primo posto
		entita_pr=org_entita_pr;
		// entita_pr punta al primo
		temp_entita_pr=org_entita_pr;
		temp_entita_pr++;
		// temp al secondo
		dist_min=distanza(entita_pr, temp_entita_pr);
		// assumo dmin tra primo e secondo
		// ora sicuramente se ent int esiste, la + vicina a O è al primo posto
		// ora riordino le entita interne , ha senso se sono almeno 3 cioè >2
		// se è una non sono qui
		// se sono due allora sono già in ordine xkè la + vicina a O è al primo posto
		if(num_ent_int>2){
			for(i=1;i<num_ent_int;i++){
				// riordino rispetto al primo elemento i=1
				// inizio for riordino entita interne
				// da effettuare 1,2,3,... n-1 cicli di riordino
				dist_min=distanza(entita_pr,temp_entita_pr);
				for(j=i+1;j<num_ent_int+1;j++){
					if(distanza(entita_pr,temp_entita_pr)<dist_min){
						dist_min=distanza(entita_pr,temp_entita_pr);
						entita_pr++;
						scambia(entita_pr,temp_entita_pr);
						entita_pr--;
					}
					temp_entita_pr++;
				}
				entita_pr++;
				temp_entita_pr=entita_pr;
				temp_entita_pr++;
			}
		}
	}
	// fine  for riordino entita interne
	// riordino entita esterne
	// definisco l'entita rispetto cui riordinare
	// se num_ent_int<>0 allora è l'ultima interna
	// sennò è il primo elemento che è già il + vicino a O
	entita_pr=org_entita_pr;
	for(i=0;i<num_ent_int;i++)
		entita_pr++;
	temp_entita_pr=entita_pr;
	// entita_pr punta all'ultima entita interna
	temp_entita_pr++;
	// temp_entita_pr punta alla prima entita esterna
	num_ent_est=num_entita-num_ent_int;
	if(num_ent_est>1){
		for(i=1;i<num_ent_est+1;i++){
			// riordino rispetto al primo elemento i=1
			// inizio for riordino entita interne
			// daeffettuare 1,2,3,... n-1 cicli di riordino
			dist_min=distanza(entita_pr,temp_entita_pr);
			for(j=i+1;j<num_ent_est+1;j++){
				if(distanza(entita_pr,temp_entita_pr)<dist_min){
					dist_min=distanza(entita_pr,temp_entita_pr);
					entita_pr++;
					scambia(entita_pr,temp_entita_pr);
					entita_pr--;
				}
				temp_entita_pr++;
			}
			entita_pr++;
			temp_entita_pr=entita_pr;
			temp_entita_pr++;
		}
	}
	entita_pr=org_entita_pr;
	temp_entita_pr=org_entita_pr;
	// ora bisogna scrivere il file riordinato di taglio
	// apro il file di taglio
	cut_file_pr=fopen(cut_file,"w");
	//file di taglio cut.cut FILE RIORDINATO DI TAGLIO
	fprintf(cut_file_pr,"FILE RIORDINATO DI TAGLIO\n");
	fprintf(cut_file_pr,"ENTITIES\n");
	fclose(cut_file_pr);
	cut_file_pr=fopen(cut_file,"a");
	input_file=fopen(nome_file_comp,"r");
	for(i=0;i<num_entita;i++){
		// recupero e scrittura ordinata delle entita secondo l'ordinazione fatta 
		// col numero di entità nel file cut.cut
		// INIZIO FOR SI I LETTURE DEL FILE NUM_ENTITA VOLTE
		for(trovato=0;trovato<1;){
			// inizio ricerca ENTITIES
			fakeint = fscanf(input_file,"%s",strinp);
			if(strncmp(strinp,"ENTITIES",8)==0)
				trovato=1;
		}
		//dovrei evitare le porcherie invisibili!!!!!!!!!!!!
		j=0;
		// contatore delle entita lette
		for(trovato=0;trovato<1;){
			// inizio for scansione entita
			// inizio for trovato entita da scrivere
			fakeint = fscanf(input_file,"%s",strinp);
			// leggo una riga del file di disegno
			// vedo che cosa è
			if(strncmp(strinp,"POLYLINE",8)==0){
				// inizio if POLYLINE
				j++;
				// aggiorno contatore entita trovate
				if(temp_entita_pr->N==j){
					// confronto con entita cercata di numero N?
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo "POLYLINE"
					for(;strncmp(strinp,"SEQEND",6);){
						// inizio for scrittura tutti i dati della polilinea
						fakeint = fscanf(input_file,"%s",strinp);
						fprintf(cut_file_pr,"%s\n",strinp);
					}
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					// cerco entita ordinata successiva
					trovato=1;
					// esco dal for scansione entita
					// fine for scrittura dati polilinea
				}
			}
			// fine if POLYLINE
			if(strncmp(strinp,"LWPOLYLINE",10)==0){
				// inizio if LWPOLYLINE
				j++;
				// aggiorno contatore entita trovate
				if(temp_entita_pr->N==j){
					// confronto con entita cercata
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo "LWPOLYLINE"
					fakeint = fscanf(input_file,"%s",strinp);
					// leggo codice
					for(;;){
						fprintf(cut_file_pr,"%s\n",strinp);
						//lo scrivo nel file
						fakeint = fscanf(input_file,"%s",strinp);
						// leggo valore
						fprintf(cut_file_pr,"%s\n",strinp);
						// lo scrivo nel file
						fakeint = fscanf(input_file,"%s",strinp);
						//leggo codice
						if(strcmp(strinp,"0")==0)
							break;
						// se è 0 esco e ho finito con la lwpoly, sennò lo scrivo
					}
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					// cerco entita ordinata successiva
					trovato=1;
					// esco dal for scansione entita
					// fine for scrittura dati lwpolilinea
				}
			}
			// fine lwpoly
			// inizio CIRCLE
			if(strncmp(strinp,"CIRCLE",6)==0){
				j++;
				if(temp_entita_pr->N==j){
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo CIRCLE
					for(;strcmp(strinp,"40");){
						// scansione CIRCLE
						fakeint = fscanf(input_file,"%s",strinp);
						fprintf(cut_file_pr,"%s\n",strinp);
						//  scrivo fino a codice 40 compreso
					}
					fakeint = fscanf(input_file,"%s",strinp);
					fprintf(cut_file_pr,"%s\n",strinp);
					//  scrivo il raggio
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					trovato=1;
					// XC YC ZC R 10 20 30 40
				}
				// fine scrittura CIRCLE
			}
			// fine if CIRCLE
			// inizio if POINT
			if(strncmp(strinp,"POINT",5)==0){
				j++;
				if(temp_entita_pr->N==j){
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo POINT
					for(;strcmp(strinp,"30");){
						// scansione POINT
						fakeint = fscanf(input_file,"%s",strinp);
						fprintf(cut_file_pr,"%s\n",strinp);
						//  scrivo fino a codice 30 compreso
					}
					fakeint = fscanf(input_file,"%s",strinp);
					fprintf(cut_file_pr,"%s\n",strinp);
					//  scrivo Z
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					trovato=1;
					// XC YC ZC 10 20 30
				}
				// fine scrittura POINT
			}
			// fine if POINT
			// inizio if ARC
			if(strncmp(strinp,"ARC",3)==0){
				j++;
				if(temp_entita_pr->N==j){
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo ARC
					for(;strcmp(strinp,"51");){
						// scansione ARC
						fakeint = fscanf(input_file,"%s",strinp);
						fprintf(cut_file_pr,"%s\n",strinp);
						//  scrivo fino a codice 51 compreso
					}
					fakeint = fscanf(input_file,"%s",strinp);
					fprintf(cut_file_pr,"%s\n",strinp);
					//  scrivo alfafin
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					trovato=1;
					// XC YC ZC R ALFAI ALFAF 10 20 30 40 50 51
				}
				// fine scrittura ARC
			}
			// fine if ARC
			// inizio if LINE
			if(strncmp(strinp,"LINE",4)==0){
				j++;
				if(temp_entita_pr->N==j){
					// scrivi entita nel file di taglio
					fprintf(cut_file_pr,"%s\n",strinp);
					// scrivo LINE
					for(;strcmp(strinp,"31");){
						// scansione LINE
						fakeint = fscanf(input_file,"%s",strinp);
						fprintf(cut_file_pr,"%s\n",strinp);
						//  scrivo fino a codice 31 compreso
					}
					fakeint = fscanf(input_file,"%s",strinp);
					fprintf(cut_file_pr,"%s\n",strinp);
					//  scrivo zf
					rewind(input_file);
					// riavvolgo il file
					temp_entita_pr++;
					trovato=1;
					// XI YI ZI XF YF ZF 10 20 30 11 21 31
				}
			}
			// fine if LINE
			if(strncmp(strinp,"EOF",3)==0){
				rewind(input_file);
				// qui non dovrei arrivarci mai !!!!
				break;
			}
		}
		// fine for trovato entita da scrivere
	}
	// fine for su i
	fprintf(cut_file_pr,"EOF\n");
	fclose(cut_file_pr);
	free(entita_pr);
}
// fine funzione
// ------------------ calcolo distanza tra entita -----------------------
// ------------------ non ottimizzata per cerchi  -----------------------
float distanza(struct entita *ent_par, struct entita *ent_arr){
	
	float xp, yp, xd, yd, dist;
	
	switch(ent_par->tipo){
		case 1:
			// polilinea
		case 2:
			// linea
		case 4:
			// arco
		case 6:
			// lwpoly
			xp=ent_par->xf;
			yp=ent_par->yf;
			break;
		case 5:
			// cerchio
		case 3:
			// punto
			xp=ent_par->xi;
			yp=ent_par->yi;
	}
	xd=ent_arr->xi;
	yd=ent_arr->yi;
	dist= (xd-xp)*(xd-xp)+(yd-yp)*(yd-yp);
	dist=sqrt(dist);
	return (dist);
}
// --------------- funzione di swap strutture -------------------
void scambia(struct entita *ent1, struct entita *ent2){
	
	struct entita entita_swap;
	
	entita_swap.N   =ent1->N;
	entita_swap.tipo=ent1->tipo;
	entita_swap.ie  =ent1->ie;
	entita_swap.xi  =ent1->xi;
	entita_swap.yi  =ent1->yi;
	entita_swap.zi  =ent1->zi;
	entita_swap.xf  =ent1->xf;
	entita_swap.yf  =ent1->yf;
	entita_swap.zf  =ent1->zf;
	entita_swap.feed  =ent1->feed;
	entita_swap.prof  =ent1->prof;
	entita_swap.ndp  =ent1->ndp;
	
	ent1->N  =ent2->N;
	ent1->tipo=ent2->tipo;
	ent1->ie  =ent2->ie;
	ent1->xi  =ent2->xi;
	ent1->yi  =ent2->yi;
	ent1->zi  =ent2->zi;
	ent1->xf  =ent2->xf;
	ent1->yf  =ent2->yf;
	ent1->zf  =ent2->zf;
	ent1->feed  =ent2->feed;
	ent1->prof  =ent2->prof;
	ent1->ndp  =ent2->ndp;
	
	ent2->N   =entita_swap.N;
	ent2->tipo=entita_swap.tipo;
	ent2->ie  =entita_swap.ie;
	ent2->xi  =entita_swap.xi;
	ent2->yi  =entita_swap.yi;
	ent2->zi  =entita_swap.zi;
	ent2->xf  =entita_swap.xf;
	ent2->yf  =entita_swap.yf;
	ent2->zf  =entita_swap.zf;
	ent2->feed  =entita_swap.feed;
	ent2->prof  =entita_swap.prof;
	ent2->ndp  =entita_swap.ndp;
}
//===================================================================================================
void taglia(void){
	
	//char tasto;  // WEB fixed unused variable warning
	unsigned int trovato,  leggi;
	
	ent_in_esame=0;
	x=y=xr=yr=0;
	z=zr=dp; 
	// ----------------- modifica per scrivere file CNC ----------------------------------
	log_file=fopen(cut_log_file,"w");
	if(log_file==NULL){
		printf("\n\nUnable to open cut log file\n\n");
		return;
	}
	*cnc_file='\0';  // WEB fixed mkissing cast warning
	strncpy(cnc_file,nome_file_comp, sizeof(cnc_file)-1);
	cnc_file[sizeof(cnc_file)-1] = '\0';
	// per evitare un overflow calcolo la dimensione massima libera
	unsigned int free_space = sizeof(cnc_file) - strlen(".nc") -1; // -1 per togliere anche lo \0 finame di .nc
	strncat(cnc_file,".nc", free_space);
	output_file_cnc=fopen(cnc_file,"w");
	if(output_file_cnc==NULL){
		*cnc_file='\0';  // WEB fixed missing cast warning
		printf("\n\nUnable to open output cnc cut file\n\n");
		return;
	}
	fprintf(output_file_cnc, "(File in codice G)\n");
	if(strcmp(unita,"i")==0)
		fprintf(output_file_cnc, "G20\n");
	else
		fprintf(output_file_cnc, "G21\n");
	fprintf(output_file_cnc, "%s\n", modalita);
	fprintf(output_file_cnc, "F%d\n", vt);
	fprintf(output_file_cnc, "G0 X0 Y0 Z%f\n", dp); //alzo l'utensile
	fprintf(output_file_cnc, "G90\n"); //imposto spostamenti assoluti
	//----------------------------------------------------------------
	input_file=fopen(cut_file,"r"); //leggo file ordinato cut.cut
	if(input_file==NULL){
		printf("\n\nUnable to open input file\n\n");
		return;
	}
	//************** taglio profili interni
	stepx_f=stepy_f=stepz_f=0;
	for(trovato=0;trovato<1;){
		fakeint = fscanf(input_file,"%s",stringa);
		if(strncmp(stringa,"ENTITIES",8)==0)
			trovato=1;
	} //dovrei evitare le porcherie invisibili!!!!!!!!!!!!
	fprintf(log_file,"Inizio parsing entità interne da tagliare\n\n");
	leggi=1;
	for(;;){
		fprintf(log_file,"Stringa=%s\n", stringa);
		fprintf(log_file,"Leggi=%d\n", leggi);
		if(leggi==1){
			fakeint = fscanf(input_file,"%s",stringa);
			fprintf(log_file,"Stringa letta=%s\n", stringa);
		}
		else
			leggi=1;
		if(strncmp(stringa,"EOF",3)==0){
			break;
		}
		// vedo se il comando e' una poliline
		if(strncmp(stringa,"POLYLINE",8)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata polilinea. Entità %d\n", ent_in_esame);
			cut_polyline(0, np);  //interno 0
		}
		if(strncmp(stringa,"CIRCLE",6)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovato cerchio. Entità %d\n", ent_in_esame);
			cut_circle(0, np);  //interno 0
		}
		if(strncmp(stringa,"POINT",5)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata punto. Entità %d\n", ent_in_esame);
			cut_point(0, dwell, np);  //interno 0
		}
		if(strncmp(stringa,"ARC",3)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata arco. Entità %d\n", ent_in_esame);
			cut_arc(0, np);  //interno 0
		}
		if(strncmp(stringa,"LINE",4)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata linea. Entità %d\n", ent_in_esame);
			cut_line(0, np);  //interno 0
		}
		if(strncmp(stringa,"LWPOLYLINE",10)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata lwpoly. Entità %d\n", ent_in_esame);
			cut_lwpolyline(0, np);
			leggi=0;  //interno 0
		}
	}
	//*********   taglio profilo esterno
	rewind(input_file);
	for(trovato=0;trovato<1;){
		fakeint = fscanf(input_file,"%s",stringa);
		if(strncmp(stringa,"ENTITIES",8)==0)
			trovato=1;
	} //dovrei evitare le porcherie invisibili!!!!!!!!!!!!
	ent_in_esame=0;;
	fprintf(log_file,"\nInizio parsing entità esterne da tagliare\n\n");
	leggi=1;
	for(;;){
		fprintf(log_file,"Stringa=%s\n", stringa);
		fprintf(log_file,"Leggi=%d\n", leggi);
		if(leggi==1){
			fakeint = fscanf(input_file,"%s",stringa);
			fprintf(log_file,"Stringa letta=%s\n", stringa);
		}
		else
			leggi=1;
		if(strncmp(stringa,"EOF",3)==0){
			break;
		}
		// vedo se il comando e' una poliline
		if(strncmp(stringa,"POLYLINE",8)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata polilinea. Entità %d\n", ent_in_esame);
			cut_polyline(7, np);  //interno 0
		}
		if(strncmp(stringa,"CIRCLE",6)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata cerchio. Entità %d\n", ent_in_esame);
			cut_circle(7, np);  //interno 0
		}
		if(strncmp(stringa,"POINT",5)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata punto. Entità %d\n", ent_in_esame);
			cut_point(7, dwell, np);  //interno 0
		}
		if(strncmp(stringa,"ARC",3)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata arco. Entità %d\n", ent_in_esame);
			cut_arc(7, np);  //interno 0
		}
		if(strncmp(stringa,"LINE",4)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata linea. Entità %d\n", ent_in_esame);
			cut_line(7, np);  //interno 0
		}
		if(strncmp(stringa,"LWPOLYLINE",10)==0){
			ent_in_esame++;
			fprintf(log_file,"Trovata lwpoly. Entità %d\n", ent_in_esame);
			cut_lwpolyline(7, np);
			leggi=0;
		}
	}
	fclose(input_file);
	fprintf(output_file_cnc,"G0 X0 Y0\n");
	sposta_punta(-x,-y,0,100,0);
	fprintf(output_file_cnc,"M5\n");
	fprintf(output_file_cnc,"M2\n");
	fclose(output_file_cnc);
	fclose(log_file);
	return;
}
// ******************** SPOSTA PUNTA ***************************************
void sposta_punta(float ddx, float ddy, float ddz, int speed, int modot){
	x=x+ddx;
	y=y+ddy;
	z=z+ddz;
	xr=x;
	yr=y;
	zr=z;
}
#ifdef NEED_ROUND  // WEB took out function round to avoid redefinition
// ************************************************************************
long round(float f){
	if (f>0){
		if ((f-(long)f)<=0.5){
			return (f);
		}
		else{
			return ((long)f+1);
		}
	}
	else if (f<0){
		if ((f-(long)f)>-0.5){
			return ((long)f);
		}
		else{
			return ((long)f-1);
		}
	}
	else{
		return 0;
	}
}
#endif  // WEB endif
//*****************************************************
float abso_float(float value){
	if(value>=0)
		return(value);
	else
		return(-value);
}
//*****************************************************
long abso_long(long value){
	if(value>=0)
		return(value);
	else
		return(-value);
}
//**********************************************
void cut_polyline(int colore, int volte){
	// colore=7 taglio esterno colore=0 taglio interno volte = np dei default
	int aperta=1;
	// polilinea aperta di default
	int nvp=0, i, color_line, volte_org;
	char file_temp[]="router.tmp";
	FILE *temp_file;
	unsigned char j, tagliare=1;
	float xd, yd, zd, dx, dy, dz, x0, y0, z0, bulge_now, bulge_next;
	float xip, yip, zip, bulge_next_ip, zattuale;
	float pp_orig, vt_orig;
	
	nvp=0;
	temp_file=fopen(file_temp,"w");
	for(;;){
		// inizio for scansione polilinea
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------- 1.0.4 ----------------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if((strncmp(strinp,"8",1)==0)&(nvp==0)){
			// solo fuori da vertex ossia nvp = 0
			evaluate_layer_name();
		}
		// --------------------- 1.0.4 ----------------------------------------------
		// vedo se la poli e' chiusa o aperta valutando il codice
		// 70 prima dei VERTEX
		if((strncmp(strinp,"70",2)==0)&(nvp==0)){
			// solo fuori da vertex ossia nvp = 0
			fakeint = fscanf(input_file,"%s",strinp);
			aperta = (atoi(strinp) & 1)? 0 : 1;
			//if((strncmp(strinp,"1",1)==0)|(strncmp(strinp,"3",1)==0))
			//	aperta=0;
			// allora la polilinea e' chiusa
			// dovro' aggiungere un vertice coinc
			// col primo nel file di taglio temporaneo
			//if((strncmp(strinp,"2",1)==0)|(strncmp(strinp,"3",1)==0)
		}
		// vedo se ho trovato un vertice
		if(strncmp(strinp,"VERTEX",6)==0){
			// si quindi incremento il numero di vertici nvp
			nvp++;
			for(;;){
				// inizio for scansione interna vertice
				fakeint = fscanf(input_file,"%s",strinp);
				// valuto se il codice letto Š 10 cioŠ x del Vertex
				if(strcmp(strinp,"10")==0){
					// se si allora seguono 20 e 30 cioŠ xy e z
					fakeint = fscanf(input_file,"%f",&xd);
					fakeint = fscanf(input_file,"%s",strinp); // 20
					fakeint = fscanf(input_file,"%f",&yd);    // Y
					fakeint = fscanf(input_file,"%s",strinp); // 30 ?
					//======== Fix here by Ray Abram ===========
					if (strncmp(strinp,"30",2)==0){
						fakeint = fscanf(input_file,"%f",&zd);    // Z
						fakeint = fscanf(input_file,"%s",strinp); // read next command    42?
					}
					else{
						zd = 0;
					}
					//===========================================
					//vedo se il codice che segue la Z vale 42 ossia raccordo
					if(strncmp(strinp,"42",2)==0)
						// se si allora leggo la tang dell'angolo/4 dell'arco
						// con cui raggiungere il verice che segue
						fakeint = fscanf(input_file,"%f",&bulge_next);
					else
						// altrimenti non c'Š raccordo
						bulge_next=0;
					// scrivo i dati del vertice nel file temporaneo di taglio
					fprintf(temp_file,"%f\n%f\n%f\n%f\n",xd,yd,zd,bulge_next);
					break;
					// esco dal for di scansione interno al vertice
				}
				else if (strncmp(strinp,"EOF",3)==0){
					printf("\nFile anomalo!");
					err_ch=fcloseall();
					return;
				}
			}
			// fine for scansione interna vertice
		}
		// vedo se sono finiti i vertici
		if(strncmp(strinp,"SEQEND",6)==0){
			// inizio if SEQEND
			// se si chiudo il file temporaneo di taglio
			fclose(temp_file);
			// vedo se la polilinea Š chiusa
			if(aperta==0){
				// se la polilinea e' chiusa
				//devo aggiungere un ultimo punto coinc col primo
				// se Š chiusa riapro il file temporaneo di taglio in lettura
				temp_file=fopen(file_temp,"r");
				// leggo la x, la y, la z e alfa/4 del vertice di partenza
				fakeint = fscanf(temp_file,"%f",&x0);
				fakeint = fscanf(temp_file,"%f",&y0);
				fakeint = fscanf(temp_file,"%f",&z0);
				fakeint = fscanf(temp_file,"%f",&bulge_next);
				// richiudo il file temporaneo di taglio
				fclose(temp_file);
				// lo riapro in append
				temp_file=fopen(file_temp,"a");
				// appendo le coordinate del primo punto alla fine
				fprintf(temp_file,"%f\n%f\n%f\n%f\n",x0,y0,z0,bulge_next);
				// chiudo il file
				fclose(temp_file);
				// incremento il numero di vertici della polilinea
				nvp++;
			}
			break;
			// finita scansione polilinea.
			// esco dal for scansione polilinea
		}
		// fine if SEQEND
		if(strcmp(strinp,"62")==0){
			// verifico il codice colore della polilinea per decidere
			// se procedere al taglio
			fakeint = fscanf(input_file,"%d",&color_line);
			if(colore==7){
				// se il colore Š 7 allora devo tagliare poli esterne
				if(color_line==colore)
					// devo verificare se Š da tagliare o no
					tagliare=1;
				else
					tagliare=0;
			}
			if(colore==0){
				// devo tagliare polilinee di colore <> da 7 interne
				if(color_line!=7)
					tagliare=1;
				else
					tagliare=0;
			}
		}
	}
	// fine for scansione polilinea
	if(tagliare==1){
		// se la poli è da tagliare allora procedo
		temp_file=fopen(file_temp,"r");
		fakeint = fscanf(temp_file,"%f",&xd);
		fakeint = fscanf(temp_file,"%f",&yd);
		fakeint = fscanf(temp_file,"%f",&zd);
		fakeint = fscanf(temp_file,"%f",&bulge_next);
		fprintf(output_file_cnc,"G0 X%f Y%f\n",xd, yd);
		dx=xd-xr;
		dy=yd-yr;
		// posizionamento inizio polilinea
		sposta_punta(dx,dy,0,100,0);
		// spostamento all'inizio della polilinea
		if(zd<0){
			 // allora taglio esattamente a quella profondità
			pp_orig=pp;
			pp=-zd;
			volte_org=volte;
			volte=1;
			// in una passata singola
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){
			// allora l'entità ha parametri di taglio propri
			pp_orig=pp;
			pp=dp_entity;
			volte_org=volte;
			volte=np_entity;
			vt_orig=vt;
			vt=f_entity;
			fprintf(output_file_cnc, "F%d\n", vt);
		}
		//++++++++++++++++++++ 1.0.4 +++++++++++++++++++++++++++++++++++++++++++++++++
		dz=dit-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		sposta_punta(0,0,dz,100,0);
		// abbassamento punta a inizio taglio
		fprintf(output_file_cnc,"G1 Z0\n");
		dz=-pp-zr;
		z=0;
		bulge_now=bulge_next;
		for(j=0;j<volte;j++){
			// inizio ciclo da ripetere per volte=np passate
			z=-pp*(j+1);
			fprintf(output_file_cnc,"G1 Z%f\n",z);
			sposta_punta(0,0,dz,vt,1);
			// abbassamento punta di pp
			for(i=1;i<nvp;i++){
				// inizio scansione vertici della polilinea
				fakeint = fscanf(temp_file,"%f",&xd);
				fakeint = fscanf(temp_file,"%f",&yd);
				fakeint = fscanf(temp_file,"%f",&zd);
				fakeint = fscanf(temp_file,"%f",&bulge_next);
				dx=xd-xr;
				dy=yd-yr;
				if(bulge_now==0){
					// segmento di retta?
					fprintf(output_file_cnc,"G1 X%f Y%f\n", xd, yd);
					sposta_punta(dx,dy,0,vt,1);
				}
				else{
					// arco
					cut_arco(dx,dy,bulge_now);
				}
				bulge_now=bulge_next;
			}
			// fine scansione vertici della polilinea
			if (j!=volte-1){
				// ultima passata?
				// no! quindi devo tornare all'inizio della polilinea
				rewind(temp_file);
				dz=-pp;
				// recupero coord punto iniziale polilinea
				fakeint = fscanf(temp_file,"%f",&xip);
				fakeint = fscanf(temp_file,"%f",&yip);
				fakeint = fscanf(temp_file,"%f",&zip);
				fakeint = fscanf(temp_file,"%f",&bulge_next_ip);
				bulge_now=bulge_next_ip;
				if((xip!=xd)&&(yip!=yd)&&(volte>1)){
					// polilinea chiusa?
					// no!
					zattuale=zr;
					// memorizzo a che z sono
					fprintf(output_file_cnc,"G0 Z%f\n", dp);
					sposta_punta(0,0,-zr+dit,100,0);
					// alzo la punta
					dx=xip-xr;
					// calcolo gli spostamenti
					dy=yip-yr;
					fprintf(output_file_cnc,"G0 X%f Y%f\n", xip, yip);
					sposta_punta(dx,dy,0,100,0);
					// torno a casa
					fprintf(output_file_cnc,"G0 Z%f\n", dit);
					fprintf(output_file_cnc,"G1 Z0\n");
					sposta_punta(0,0,zattuale-zr,vt,1);
					// torno alla profondit… cui mi trovavo
				}
			}
			// ultima passata?
		}
		// fine del ciclo da ripetere per np passate
		fclose(temp_file);
		dz=dp-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dp);
		sposta_punta(0,0,dz,100,0);
		//++++++++++++++++++++
		if(zd<0){
			pp=pp_orig;
			volte=volte_org;
		}
		//++++++++++++++++++++
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){
			// allora l'entità ha parametri di taglio propri
			pp=pp_orig;
			// rimetto la prof di passata al valore di default
			volte=volte_org;
			vt=vt_orig;
			ent_con_par_propri = 0;
			fprintf(output_file_cnc, "F%d\n", vt);
			// rimetto il feed al valore di default
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++
	  }
	  // fine if poliline
}
//*************************************
void cut_circle(int colore, int volte){
	
	int  color_line, volte_org, vt_orig;
	unsigned char j, tagliare=1, coor_ok=0, col_ok=0;
	float xpo, ypo, xc, yc, zc, r, beta, dx, dy, dz;
	float pp_orig;
	
	ent_con_par_propri = 0;
	
	fprintf(log_file,"Inizio scanning Cerchio\n");
	for(;;){
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------- 1.0.4 ------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if((strncmp(strinp,"8",1)==0)){ // codice 8 nomelayer
			evaluate_layer_name();
		}
		// --------------------- 1.0.4 ------------------------------------
		fprintf(log_file,"linea 1740 strinp=%s\n", strinp);
		if(strcmp(strinp,"10")==0){
			fakeint = fscanf(input_file,"%f",&xc);    // coordinata x del centro
			fakeint = fscanf(input_file,"%s",strinp); // codice 20 coord y del centro
			fakeint = fscanf(input_file,"%f",&yc);
			fakeint = fscanf(input_file,"%s",strinp); // codice 30 coord z del centro
			fakeint = fscanf(input_file,"%f",&zc);    //
			fakeint = fscanf(input_file,"%s",strinp); // codice 40 raggio 
			fakeint = fscanf(input_file,"%f",&r);    //
			coor_ok=1;
			if(col_ok==1)
				break;
		}
		// vedo il colore dell'entita per vedere se deve essere tagliata ora o no
		if(strcmp(strinp,"62")==0){ // codice 62 colore entita
			fakeint = fscanf(input_file,"%d",&color_line);
			if(colore==7){
				if(color_line==colore)
					tagliare=1;
				else
					tagliare=0;
			}
			if(colore==0){
				if(color_line!=7)
					tagliare=1;
				else
					tagliare=0;
			}
			col_ok=1;
			if(coor_ok==1)
				break;
		}
	}
	if(tagliare==1){
		//dx=xc-x;
		//dy=yc-y; // distanze da percorrere per raggiungere il centro del cerchio
		//------------------------------------------------------ RIVEDERE
		/*if((dx!=0)||(dy!=0)){
			// se non sono sul centro
			if(dy==0){
				if(dx>0)
					beta=pi/2;
				else
					beta=-pi/2;
			}
			if(dy!=0)
				beta=atan2(dy,dx);
			xpo=xc-r*cos(beta);
			ypo=yc-r*sin(beta);
			dx=xpo-xr;
			dy=ypo-yr;
		}
		else{
			// sono proprio sul centro
			dx=-r;
			dy=0;
			xpo=xc-r;
			ypo=yc;
		}
		*/
		// provo a mettere un punto inizio taglio a 180 gradi senza calcolare intersezioni
		xpo = xc - r;
		ypo = yc;
		dx = xpo-x;
		dy = ypo-y; // distanze da percorrere per raggiungere il centro del cerchio
		// e se sono dentro il cerchio?
		fprintf(output_file_cnc,"G0 X%f Y%f\n", xpo, ypo);
		sposta_punta(dx,dy,0,100,0);
		//------------------------------------------------------- RIVEDERE
		if(zc<0){
			pp_orig=pp;
			pp=-zc;
			volte_org=volte;
			volte=1;
		}
		dz=dit-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		sposta_punta(0,0,dz,100,0);
		//++++++++++++++++++++ 1.0.4 +++++++++++++++++++++++++++++++++++++++++++++++
		 if(ent_con_par_propri==1){
			 // allora l'entità ha parametri di taglio propri
			 pp_orig=pp;
			 pp=dp_entity;
			 volte_org=volte;
			 volte=np_entity;
			 vt_orig=vt;
			 vt=f_entity;
			 fprintf(output_file_cnc, "F%d\n", vt);
		 }
		 //++++++++++++++++++++ 1.0.4 +++++++++++++++++++++++++++++++++++++++++++++++
		 dz=-pp-zr;
		 dx=2*r*cos(beta);
		 dy=2*r*sin(beta);
		 fprintf(output_file_cnc,"G1 Z0\n");
		 z=0;
		 fprintf(output_file_cnc,"(Cerchio in %d passate)\n",volte);
		 for(j=0;j<volte;j++){
			 z=-pp*(j+1);
			 fprintf(output_file_cnc,"G1 Z%f\n", z);
			 sposta_punta(0,0,dz,vt,1);
			 //cut_arco(dx,dy,1); // qui taglio il cerchio con 2 archi
			 //cut_arco(-dx,-dy,1);
			 // metto codice g2 o g3 con i e j cerchio completo
			 fprintf(output_file_cnc,"G91\n");
			 fprintf(output_file_cnc,"G2 X0 Y0 I%6.5f J0.00000\n", r);
			 //fprintf(output_file_cnc,"G2 X%6.5f Y%6.5f Z0 I%6.5f J%6.5f\n", 0, 0,  xc-xpo, yc-ypo);
			 fprintf(output_file_cnc,"G90\n");
			 dz=-pp;
		 }
		 dz=dp-zr;
		 fprintf(output_file_cnc,"G0 Z%f\n", dp);
		 z=dp;
		 sposta_punta(0,0,dz,100,0);
		 if(zc<0){
			 pp=pp_orig;
			 volte=volte_org;
		 }
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	   if(ent_con_par_propri==1){
		   // allora l'entità ha parametri di taglio propri
		   pp=pp_orig;
		   // rimetto la prof di passata al valore di default
		   volte=volte_org;
		   vt=vt_orig;	
		   ent_con_par_propri = 0;
		   fprintf(output_file_cnc, "F%d\n", vt);
		   // rimetto il feed al valore di default
	   }
	}
}
//**********************************************
void cut_lwpolyline(int colore, int volte){ 
	// colore=7 taglio esterno colore=0 taglio interno
	int aperta=1, ok;
	// polilinea aperta di default
	int nvp, i, j, color_line, volte_org;
	char file_temp[]="router.tmp";
	FILE *temp_file;
	float pp_orig, vt_orig;
	unsigned char tagliare=1;
	float xd, yd, dx, dy, dz, x0, y0, bulge_now, bulge_next;
	float xip, yip, bulge_next_ip, zattuale;

	temp_file=fopen(file_temp,"w");
	nvp=0;
	fprintf(log_file,"\nEntità in esame%d\n", ent_in_esame);

	for(;;){// inizio for scansione lwpolilinea ATTENZIONE... per evitare casini con codici e numeri 
			// modificare la scansione leggendo sempre coppie di dati!!!!!! 
		fakeint = fscanf(input_file,"%s",strinp);
		ok=0;
		fprintf(log_file,"Scanning codici lwpolyline. Entità strinp=%s\n", strinp);
		// --------------------- 1.0.4 ----------------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if(strncmp(strinp,"8",1)==0){
			evaluate_layer_name();
			ok=1;//chiusa seq codice/valore
		}
		// --------------------- 1.0.4 ----------------------------------------------
        // the number of vertex in lwpolyline is tagged by code 90
		if(strncmp(strinp,"90",2)==0){
			fakeint = fscanf(input_file,"%d",&nvp);
			fprintf(log_file,"nvp=%d\n", nvp);
			ok=1; //chiusa seq codice/valore
		}
		if(strcmp(strinp,"62")==0){	// verifico il codice colore della polilinea per decidere
        							// se procedere al taglio
			fakeint = fscanf(input_file,"%d",&color_line);
			fprintf(log_file,"Colore lwpolyline. color_line=%d\n", color_line);
			ok=1;
			if(colore==7){// se il colore Š 7 allora devo tagliare poli esterne
				if(color_line==colore)// devo verificare se Š da tagliare o no
					tagliare=1;
				else
				tagliare=0;
			}
			if(colore==0) {// devo tagliare polilinee di colore <> da 7 interne
				if(color_line!=7)
					tagliare=1;
				else
					tagliare=0;
			}
			fprintf(log_file,"Tagliare=%d\n", tagliare);
		}
		if((strcmp(strinp,"10")==0)&(nvp!=0)){
			for(i=0;i<nvp;i++){  // inizio for scansione interna vertice
				fakeint = fscanf(input_file,"%f",&xd);
				fakeint = fscanf(input_file,"%s",strinp); // 20
				fakeint = fscanf(input_file,"%f",&yd);    // Y
				fakeint = fscanf(input_file,"%s",strinp); // 42?
				//vedo se il codice che segue la Z vale 42 ossia raccordo 0 no raccordo, > 0 arco antiorario, < 0 arco orario
				// rappresenta 1/4 dell'angolo di apertura in radianti
				if(strncmp(strinp,"42",2)==0){ // può essere 42, 10, EOF, o altro
					// se si allora leggo la tang dell'angolo/4 dell'arco
					// con cui raggiungere il verice che segue
					fakeint = fscanf(input_file,"%f",&bulge_next);
					fakeint = fscanf(input_file,"%s",strinp);
				}
				else
					bulge_next=0;// altrimenti non c'Š raccordo
					// scrivo i dati del vertice nel file temporaneo di taglio
				fprintf(temp_file,"%f\n%f\n%f\n",xd,yd,bulge_next);
			}
			strncpy(stringa,strinp, sizeof(stringa)-1);
			stringa[sizeof(stringa)-1] = '\0';
			// ho terminato la lettura del cut.cut
			break;
		}
		fprintf(log_file,"Verifico se strinp = 70 strinp=%s\n", strinp);
		if(strcmp(strinp,"70")==0){ // solo fuori da vertex
			fakeint = fscanf(input_file,"%s",strinp);
			fprintf(log_file,"lwpolyline. codice 70=%s\n", strinp);
			ok=1;
			aperta = (atoi(strinp) & 1)? 0 : 1;
			//if((strncmp(strinp,"1",1)==0)|(strncmp(strinp,"3",1)==0))
			//	aperta=0;	// allora la polilinea e' chiusa
			// dovro' aggiungere un vertice coinc
			// col primo nel file di taglio temporaneo
			fprintf(log_file, "Rilevata lwpolyline chiusa line 1955\n");
		}
		if (strncmp(strinp,"EOF",3)==0){
			printf("\nFile anomalo! In riga sorgente 2200 della ver %s", ver);
			err_ch=fcloseall();
			return;
		}
		if(ok==0)
			fakeint = fscanf(input_file,"%s",strinp);
	}	// fine for scansione interna
	// vedo se sono finiti i vertici
	// se si chiudo il file temporaneo di taglio
	fclose(temp_file);
	// vedo se la polilinea Š chiusa
	if(aperta==0){	// se la polilineae'chiusadevo aggiungere un ultimo punto coinc col primo se non coincidono
		// se Š chiusa riapro il file temporaneo di taglio in lettura
		temp_file=fopen(file_temp,"r");
		// leggo la x, la y, la z e alfa/4 del vertice di partenza
		fakeint = fscanf(temp_file,"%f",&x0);
		fakeint = fscanf(temp_file,"%f",&y0);
		fakeint = fscanf(temp_file,"%f",&bulge_next);
		// richiudo il file temporaneo di taglio
		fclose(temp_file);
		if((x0!=xd)|(y0!=yd)){
			// lo riapro in append
			temp_file=fopen(file_temp,"a");
			// appendo le coordinate del primo punto alla fine
			fprintf(temp_file,"%f\n%f\n%f\n",x0,y0,bulge_next);
			fprintf(log_file,"Aggiunto punto alla fine del file temporaneo di taglio lwpolyline\n");
			// chiudo il file
			fclose(temp_file);
			nvp++;
		}
	}
	// il file cut.cut è stato scritto
	if(tagliare==1){
		// se la poli Š da tagliare allora procedo
		temp_file=fopen(file_temp,"r");
		fakeint = fscanf(temp_file,"%f",&xd);
		fakeint = fscanf(temp_file,"%f",&yd);
		fakeint = fscanf(temp_file,"%f",&bulge_next);
		fprintf(output_file_cnc,"G0 X%f Y%f\n",xd, yd);
		dx=xd-xr;dy=yd-yr;
		// posizionamento inizio polilinea
		sposta_punta(dx,dy,0,100,0);  // spostamento all'inizio della polilinea
		dz=dit-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		sposta_punta(0,0,dz,100,0);  // abbassamento punta a inizio taglio
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp_orig=pp;
			pp=dp_entity;
			volte_org=volte;
			volte=np_entity;
			vt_orig=vt;
			vt=f_entity;
			fprintf(output_file_cnc, "F%d\n", vt);
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		fprintf(output_file_cnc,"G1 Z0\n");
		dz=-pp-zr;
		bulge_now=bulge_next;
		fprintf(output_file_cnc,"(Lwpolyline in %d passate)\n",volte);
		for(j=0;j<volte;j++){  // inizio ciclo da ripetere per volte=np passate
			z=-pp*(j+1);
			fprintf(output_file_cnc,"G1 Z%f\n",z);
			sposta_punta(0,0,dz,vt,1);  // abbassamento punta di pp
			for(i=1;i<nvp;i++){ // inizio scansione vertici della polilinea
				fakeint = fscanf(temp_file,"%f",&xd);
				fakeint = fscanf(temp_file,"%f",&yd);
				fakeint = fscanf(temp_file,"%f",&bulge_next);
				dx=xd-xr;
				dy=yd-yr;
				// bulge_now è la tangente di (theta / 4)) angolo di apertura dell'arco
				// ovvero il bulge
				if(bulge_now==0){ // segmento di retta?
					fprintf(output_file_cnc,"G1 X%f Y%f\n", xd, yd);
					sposta_punta(dx,dy,0,vt,1);
				}
				else {                  // arco
					cut_arco(dx,dy,bulge_now);
				}
				bulge_now=bulge_next;
			}
			// fine scansione vertici della polilinea
			if(j!=(volte-1)){  // ultima passata?
				// no! quindi devo tornare all'inizio della polilinea
				rewind(temp_file);
				dz=-pp;
				// recupero coord punto iniziale polilinea
				fakeint = fscanf(temp_file,"%f",&xip);
				fakeint = fscanf(temp_file,"%f",&yip);
				fakeint = fscanf(temp_file,"%f",&bulge_next_ip);
				bulge_now=bulge_next_ip;
				if(((xip!=xd)|(yip!=yd))&&(volte>1)){ // polilinea chiusa? rivedere ... OR
					// no!
					zattuale=zr;                    // memorizzo a che z sono
					fprintf(output_file_cnc,"G0 Z%f\n", dp);
					sposta_punta(0,0,-zr+dit,100,0);   // alzo la punta
					dx=xip-xr;                      // calcolo gli spostamenti
					dy=yip-yr;
					fprintf(output_file_cnc,"G0 X%f Y%f\n", xip, yip);
					sposta_punta(dx,dy,0,100,0);       // torno a casa
					fprintf(output_file_cnc,"G0 Z%f\n", dit);
					fprintf(output_file_cnc,"G1 Z0\n");
					sposta_punta(0,0,zattuale-zr,vt,1); // torno alla profondit… cui mi trovavo
				}
			}
			// ultima passata?
		}
		// fine del ciclo da ripetere per np passate
		fclose(temp_file);
		dz=dp-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dp);
		sposta_punta(0,0,dz,100,0);
		if(ent_con_par_propri==1){
			// allora l'entità ha parametri di taglio propri
			pp=pp_orig;
			// rimetto la prof di passata al valore di default
			volte=volte_org;
			vt=vt_orig;
			ent_con_par_propri = 0;
			fprintf(output_file_cnc, "F%d\n", vt);
			// rimetto il feed al valore di default
		}
	}
	// fine if lwpoliline
	return;
}
//********************* foratura *****************************
void cut_point(int colore,float attesa, int volte){ //prevedere il rompitruciolo
	
	int  color_point, volte_org, j;
	unsigned char tagliare=1;
	float xc, yc, zc, dx, dy, dz, pp_orig, vt_orig;
	
	for(;;){
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------- 1.0.4 ----------------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if((strncmp(strinp,"8",1)==0)){
			evaluate_layer_name();
		}
		// --------------------- 1.0.4 ----------------------------------------------
		if(strcmp(strinp,"10")==0){
			fakeint = fscanf(input_file,"%f",&xc);
			fakeint = fscanf(input_file,"%s",strinp); // 20
			fakeint = fscanf(input_file,"%f",&yc);
			fakeint = fscanf(input_file,"%s",strinp); // 30
			fakeint = fscanf(input_file,"%f",&zc);    //
			break;
		}
		if(strcmp(strinp,"62")==0){
			fakeint = fscanf(input_file,"%d",&color_point);
			if(colore==7){
				if(color_point==colore)
					tagliare=1;
				else
					tagliare=0;
			}
			if(colore==0){
				if(color_point!=7)
					tagliare=1;
				else
					tagliare=0;
			}
		}
	}
	if(tagliare==1){
		dx=xc-xr;
		dy=yc-yr;
		fprintf(output_file_cnc,"G0 X%f Y%f\n", xc, yc);
		sposta_punta(dx,dy,0,100,0);
		//1.4.0 ------------------------------------------------------------------- inizio
		if(zc<0){
			pp_orig=pp;
			pp=-zc;
			volte_org=volte;
			volte=1;
		}
		dz=dit-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		sposta_punta(0,0,dz,100,0);
//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp_orig=pp;
			pp=dp_entity;
			volte_org=volte;
			volte=np_entity;
			vt_orig=vt;
			vt=f_entity;
			fprintf(output_file_cnc, "F%d\n", vt);
		}
//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		dz=-pp-zr;
		fprintf(output_file_cnc,"G1 Z0\n");
		z=0;
		fprintf(output_file_cnc,"(Foro in %d passate)\n",volte);
		for(j=0;j<volte;j++){
			z=-pp*(j+1);
			fprintf(output_file_cnc,"G1 Z%f\n", z);
			sposta_punta(0,0,z,vt,1);
			fprintf(output_file_cnc,"G0 Z0\n");
			sposta_punta(0,0,-z,vt,1);
		}
		fprintf(output_file_cnc,"G0 Z%f\n", dp);
		z=dp;
		sposta_punta(0,0,z,100,0);
		if(zc<0){
			pp=pp_orig;
			volte=volte_org;
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp=pp_orig;// rimetto la prof di passata al valore di default
			volte=volte_org;
			vt=vt_orig;	
			ent_con_par_propri = 0;
			fprintf(output_file_cnc, "F%d\n", vt); // rimetto il feed al valore di default
		}
		//1.4.0 ------------------------------------------------------------------- fine
	}
}
//************************* taglio arco *************************
void cut_arco(float dx, float dy, float bulge){
	// viene richiamata da cut arc entita dxf
	//
	float popf, r, xm, ym, xc, yc, delta, d, dalfa, alfapart, beta, alfa;
	float xpi, ypi, dxi, dyi, xarc, yarc;
	int i, nseg, orario;
	//
	xarc=yarc=0;					// coordinate relative PO origine arco
	alfa=4*atan(bulge);				// angolo di apertura dell'arco in radianti da bulge
									// bulge varia da 0 a 1 ovvero l'angolo da 0 a 45 quindi alfa da 0 a 180
									// antiorario bulge > 0 -> alfa > 0
									// orario bulge < 0 -> alfa < 0
	delta=atan2(dy,dx);				// angolo inclinazione della secante - / | \ - / | \
									// atan2 da angoli nel range +/-180
	beta=(pi-abso_float(alfa))/2;	// angolo alla base triangolo po pf C
	if(alfa<0){						// se sono qui allora alfa div da zero
		orario=1;
		alfapart=pi-beta+delta;		// usato per spostare il mandrino/punto grafico secondo un arco di cerchio
									// spezzettato in segmenti
	}
	else
	{
		orario=0;
		alfapart=pi+(beta+delta);	// usato per spostare il mandrino/punto grafico un arco di cerchio
									// spezzettato in segmenti
	}
	xm=dx/2;							// punto medio segmento po pf
	ym=dy/2;
	popf=sqrt(dx*dx+dy*dy);				// lunghezza secante arco
	d = abso_float(popf/2/tan(alfa/2));	// distanza da centro arco a M
	r=sqrt(d*d+popf*popf/4);			// raggio dell'arco
	dalfa = res_arc*sqrt(resx*resx+resy*resy)/r;	// usato per spostare il mandrino/punto grafico un arco di cerchio
													// spezzettato in segmenti
	if(orario==1){
		if(abso_float(alfa)<pi){
			xc=xm+d*sin(delta);			// coordinate centro arco orario I e J rispetto al punto di partenza
			yc=ym-d*cos(delta);
			dalfa=-dalfa;				// incremento dalfa neg xke orario
			fprintf(output_file_cnc,"G91\n");
			//fprintf(output_file_cnc,"G2 X%6.5f Y%6.5f Z0 R%6.5f\n", dx, dy,  r);
			fprintf(output_file_cnc,"G2 X%6.5f Y%6.5f Z0 I%6.5f J%6.5f\n", dx, dy,  xc, yc);
			fprintf(output_file_cnc,"G90\n");
		}
		else{
			// alfa > 180 arco più lungo oltre che orario
			xc=xm-d*sin(delta);		// coordinate centro arco orario I e J
			yc=ym+d*cos(delta);
			dalfa=-dalfa;			// incremento dalfa neg xke orario
			fprintf(output_file_cnc,"G91\n");
			//fprintf(output_file_cnc,"G2 X%6.5f Y%6.5f Z0 R-%6.5f\n", dx, dy,  r);
			fprintf(output_file_cnc,"G2 X%6.5f Y%6.5f Z0 I%6.5f J%6.5f\n", dx, dy,  xc, yc);
			fprintf(output_file_cnc,"G90\n");
		}
	}
	else{ // antiorario
		if(abso_float(alfa)<pi){
			// percorso più corto
			xc=xm-d*sin(delta);
			yc=ym+d*cos(delta);
			fprintf(output_file_cnc,"G91\n");
			//fprintf(output_file_cnc,"G3 X%6.5f Y%6.5f Z0 R%6.5f\n", dx, dy,  r);
			fprintf(output_file_cnc,"G3 X%6.5f Y%6.5f Z0 I%6.5f J%6.5f\n", dx, dy,  xc, yc);
			fprintf(output_file_cnc,"G90\n");
		}
		else{ // antiorario
			// percorso lungo
			xc=xm+d*sin(delta);
			yc=ym-d*cos(delta);
			fprintf(output_file_cnc,"G91\n");
			//fprintf(output_file_cnc,"G3 X%6.5f Y%6.5f Z0 R-%6.5f\n", dx, dy,  r);
			fprintf(output_file_cnc,"G3 X%6.5f Y%6.5f Z0 I%6.5f J%6.5f\n", dx, dy,  xc, yc);
			fprintf(output_file_cnc,"G90\n");
		}
	}
	nseg=abso_float(alfa/dalfa);	//numero di segmenti in cui e diviso l'arco
	//+++++++++++++++++++++++++++++++++++
	for(i=1;i<nseg+1;i++){
		xpi=xc+r*cos(alfapart+i*dalfa);
		ypi=yc+r*sin(alfapart+i*dalfa);
		dxi=xpi-xarc;
		dyi=ypi-yarc;
		sposta_punta(dxi,dyi,0,vt,2);	//solo grafica
		xarc=xpi;
		yarc=ypi;
	}
}
//************************* taglio arc dxf *************************
void cut_arc(int colore, int volte){ // entita dxf arc
	// richiama più volte cut_arco se si devono fare passate multiple
	int  color_point, volte_org;
	unsigned char j, tagliare=1, coor_ok=0, col_ok=0, c_alfa_ok=0;
	float xc, yc, zc,  dx, dy, dz, r, alfa, alfapart, alfafine;
	float xpo, ypo, xpf, ypf, zattuale, pp_orig, vt_orig;
	
	for(;;){
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------- 1.0.4 ----------------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if((strncmp(strinp,"8",1)==0)){
			evaluate_layer_name();
		}
		// --------------------- 1.0.4 ----------------------------------------------
		if(strcmp(strinp,"10")==0){
			fakeint = fscanf(input_file,"%f",&xc);
			fakeint = fscanf(input_file,"%s",strinp);	// 20
			fakeint = fscanf(input_file,"%f",&yc);
			fakeint = fscanf(input_file,"%s",strinp);	// 30
			fakeint = fscanf(input_file,"%f",&zc);
			fakeint = fscanf(input_file,"%s",strinp);	// 40
			fakeint = fscanf(input_file,"%f",&r);
			coor_ok=1;
			if(col_ok==1)
				if(c_alfa_ok==1)
					break;
		}
		if(strcmp(strinp,"50")==0){
			fakeint = fscanf(input_file,"%f",&alfapart);	//in gradi !!!
			fakeint = fscanf(input_file,"%s",strinp); 	// 51
			fakeint = fscanf(input_file,"%f",&alfafine);	//in gradi !!!
			c_alfa_ok=1;
			if(col_ok==1)
				if(coor_ok==1)
					break;
		}
		if(strcmp(strinp,"62")==0){
			fakeint = fscanf(input_file,"%d",&color_point);
			if(colore==7){
				if(color_point==colore)
					tagliare=1;
				else
					tagliare=0;
			}
			if(colore==0){
				if(color_point!=7)
					tagliare=1;
				else
					tagliare=0;
			}
			col_ok=1;
			if(c_alfa_ok==1)
				if(coor_ok==1)
					break;
		}
	}
	alfapart=alfapart*pi/180;
	alfafine=alfafine*pi/180;
	if(tagliare==1){
		xpo=xc+r*cos(alfapart);
		ypo=yc+r*sin(alfapart);
		fprintf(output_file_cnc,"G0 X%f Y%f\n", xpo, ypo);
		xpf=xc+r*cos(alfafine);
		ypf=yc+r*sin(alfafine);
		if(alfapart<alfafine)
			alfa=alfafine-alfapart;			// arco percorso in senso antiorario
		else
			alfa=2*pi-(alfapart-alfafine);	// arco percorso in senso orario
		alfa=tan(alfa/4);
		sposta_punta(xpo-xr,ypo-yr,0,100,0);
		dx=xpf-xpo;
		dy=ypf-ypo;
		if(zc<0){
			pp_orig=pp;
			pp=-zc;
			volte_org=volte;
			volte=1;
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1) // allora l'entità ha parametri di taglio propri
		{
			pp_orig=pp;
			pp=dp_entity;
			volte_org=volte;
			volte=np_entity;
			vt_orig=vt;
			vt=f_entity;
			fprintf(output_file_cnc, "F%d\n", vt);
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		dz=dit-zr;
		sposta_punta(0,0,dz,100,0);
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		fprintf(output_file_cnc,"G1 Z0\n");
		dz=-pp-zr;
		fprintf(output_file_cnc,"(Arco con %d passate)\n",volte);
		for(j=0;j<volte;j++){
			sposta_punta(0,0,dz,vt,1);
			fprintf(output_file_cnc,"G1 Z%f\n", -pp*(j+1));
			cut_arco(dx, dy, alfa);
			if((volte>1)&(j<(volte-1))){
				zattuale=zr;
				fprintf(output_file_cnc,"G0 Z%f\n", dp);
				sposta_punta(0,0,dit-zr,100,0);
				sposta_punta(-dx,-dy,0,100,0);
				fprintf(output_file_cnc,"G0 X%f Y%f\n", xpo, ypo);
				dz=zattuale-zr;
				sposta_punta(0,0,dz,vt,1);
				fprintf(output_file_cnc,"G0 Z%f\n", dit);
				fprintf(output_file_cnc,"G1 Z0\n");
			}
			dz=-pp;
		}
		sposta_punta(0,0,dp-zr,100,0);
		fprintf(output_file_cnc,"G0 Z%f\n", dp);
		if(zc<0){
			pp=pp_orig;
			volte=volte_org;
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp=pp_orig;// rimetto la prof di passata al valore di default
			volte=volte_org;
			vt=vt_orig;	
			ent_con_par_propri = 0;
			fprintf(output_file_cnc, "F%d\n", vt); // rimetto il feed al valore di default
		}
	}
}
//*************** taglio linee ********************************
void cut_line(int colore, int volte){
	
	int  color_point, volte_org;
	unsigned char j, tagliare=1;
	float dx, dy, dz;
	float xpo, ypo,zpo, xpf, ypf, zpf, zattuale;
	float pp_orig, vt_orig;
	
	for(;;){
		fakeint = fscanf(input_file,"%s",strinp);
		// --------------------- 1.0.4 ----------------------------------------------
		//vedo se il nome del layer racchiude parametri di taglio
		if((strncmp(strinp,"8",1)==0)){
			evaluate_layer_name();
		}
		// --------------------- 1.0.4 ----------------------------------------------
		if(strcmp(strinp,"10")==0){
			fakeint = fscanf(input_file,"%f",&xpo);
			fakeint = fscanf(input_file,"%s",strinp); // 20
			fakeint = fscanf(input_file,"%f",&ypo);
			fakeint = fscanf(input_file,"%s",strinp); // 30
			fakeint = fscanf(input_file,"%f",&zpo);
			fakeint = fscanf(input_file,"%s",strinp); // 11
			fakeint = fscanf(input_file,"%f",&xpf);
			fakeint = fscanf(input_file,"%s",strinp); // 21
			fakeint = fscanf(input_file,"%f",&ypf);
			fakeint = fscanf(input_file,"%s",strinp); // 31
			fakeint = fscanf(input_file,"%f",&zpf);
			break;
		}
		if(strcmp(strinp,"62")==0){
			fakeint = fscanf(input_file,"%d",&color_point);
			if(colore==7){
				if(color_point==colore)
					tagliare=1;
				else
					tagliare=0;
			}
			if(colore==0){
				if(color_point!=7)
					tagliare=1;
				else
					tagliare=0;
			}
		}
	}
	if(tagliare==1){
		fprintf(output_file_cnc,"G0 X%f Y%f\n", xpo, ypo);
		sposta_punta(xpo-xr,ypo-yr,0,100,0);
		dx=xpf-xpo;
		dy=ypf-ypo;
		if(zpo<0){
			pp_orig=pp;
			pp=-zpo;
			volte_org=volte;
			volte=1;
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp_orig=pp;
			pp=dp_entity;
			volte_org=volte;
			volte=np_entity;
			vt_orig=vt;
			vt=f_entity;
			fprintf(output_file_cnc, "F%d\n", vt);
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++
		dz=dit-zr;
		fprintf(output_file_cnc,"G0 Z%f\n", dit);
		sposta_punta(0,0,dz,100,0);
		fprintf(output_file_cnc,"G1 Z0\n");
		dz=-pp-zr;
		fprintf(output_file_cnc,"(Line con %d passate)\n",volte);
		for(j=0;j<volte;j++){
			fprintf(output_file_cnc,"G1 Z%f\n", -pp*(j+1));
			sposta_punta(0,0,dz,vt,1);
			fprintf(output_file_cnc,"G1 X%f Y%f\n", xpf, ypf);
			sposta_punta(dx,dy,0,vt,1);
			if((volte>1)&(j<(volte-1))){
				zattuale=zr;
				fprintf(output_file_cnc,"G0 Z%f\n", dp);
				sposta_punta(0,0,dit-zr,100,0);
				fprintf(output_file_cnc,"G0 X%f Y%f\n", xpo, ypo);
				sposta_punta(-dx,-dy,0,100,0);
				dz=zattuale-zr;
				fprintf(output_file_cnc,"G0 Z%f\n", dit);
				sposta_punta(0,0,dz,vt,1);
			}
			dz=-pp;
		}
		fprintf(output_file_cnc,"G0 Z%f\n", dp);
		sposta_punta(0,0,dz,vt,1);
		sposta_punta(0,0,dp-zr,100,0);
		if(zpo<0){
			pp=pp_orig;
			volte=volte_org;
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(ent_con_par_propri==1){ // allora l'entità ha parametri di taglio propri
			pp=pp_orig;// rimetto la prof di passata al valore di default
			volte=volte_org;
			vt=vt_orig;	
			ent_con_par_propri = 0;
			fprintf(output_file_cnc, "F%d\n", vt); // rimetto il feed al valore di default
		}
		//++++++++++++++++++++ 1.0.4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}
}
// *****************************************************
void print_screen(void){
	output_file_cnc=fopen(cnc_file,"r");
	if(output_file_cnc==NULL){
		*cnc_file='\0';  // WEB fixed missing cast warning
		printf("\n\nUnable to open output cnc cut file\n\n");
		return;
	}
	while (fgets(stringa, 254, output_file_cnc) != NULL){
		printf("%s",stringa);
	}
	fclose(output_file_cnc);
}
// *****************************************************
void stampa_help(void){
	printf("\n Example:\n dxf2G -n 2 -p 2.5 -d 6 -i 3.5 -v 300 -u m -e \"G64 P 0.01\" -f filename.dxf \n");
	printf("\n will create a G code file named filename.dxf.ncg");
	printf("\n cut in n=2 passages each p=2.5mm deep moving tool rapid at d=6mm from the piece");
	printf("\n surface, cutting feed v=300mm/min starting from i=3.5mm,");
	printf("\n using u=m metric units, ");
	printf("using e=G64 path mode smooth with 0.01mm tolerance.\n");
	printf("\nExample files are in /usr/local/share/doc/dxf2g.\n");
	printf("\n\n Default values are assumed if none given!!\n\n");
}
// *********************************************************
void evaluate_layer_name(void){
	int i;
	ent_con_par_propri=0;
	printf("Valuto nome layer\n");
	fakeint = fscanf(input_file,"%s\n",strinp);
	printf("Nome layer= %s\n", strinp);
	printf("Lunghezza nome layer= %d\n", (int)strlen(strinp));
	if(strlen(strinp)==26) {
		strncpy(layer_name,strinp,sizeof(layer_name)-1);
		layer_name[sizeof(layer_name)-1] = '\0';
		dp_layer_name_code[0]=layer_name[0];
		dp_layer_name_code[1]=layer_name[1];
		printf("dp_layer_name_code= %s\n", dp_layer_name_code);
		if(strncmp(dp_layer_name_code,"DP",2)==0){
			np_layer_name_code[0]=layer_name[11];
			np_layer_name_code[1]=layer_name[12];
			printf("np_layer_name_code= %s\n", np_layer_name_code);
			if(strncmp(np_layer_name_code,"NP",2)==0){
				f_layer_name_code[0]=layer_name[16];
				printf("f_layer_name_code= %s\n", f_layer_name_code);
				if(strncmp(f_layer_name_code,"F",1)==0){
					for(i=0;i<9;i++){
						dp_float_entity[i]=layer_name[i+2];
						f_float_entity[i]=layer_name[i+17];
					}
					for(i=0;i<3;i++){
						np_int_entity[i]=layer_name[i+13];
					}
					dp_entity=atof(dp_float_entity);
					f_entity=atof(f_float_entity);
					np_entity=atoi(np_int_entity);
					ent_con_par_propri=1;
					printf("Entita con parametri propri\n");
					printf("dp_entity=%lf\n", dp_entity);
					printf("f_entity=%lf\n", f_entity);
					printf("np_entity=%d\n", np_entity);
				}
			}
		}
	}
}

