

// Calcul de la commande en couple
#if David_couple
{
 static float qu[NA];
	static float uPD[NA], qd[NA], dqd[NA];
 //static float uStab[12];
 static float uG[NA], fG[NA], tG[NA];

	static const float FootR[3] = {0.0853f,0.f,-0.11f};
 static const float FootL[3] = {0.0853f,0.f,-0.11f};
 static const float // gains
 kp[NAMax] = { 300.f , 300.f , 300.f , 300.f , 300.f , 300.f ,   // 0 à 5  6 lignes Gains q commande en couple q
               300.f , 300.f , 300.f , 300.f , 300.f , 300.f } , // 6 à 11
 ki[NAMax] = {   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   // 0 à 5
                 1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f } , // 6 à 11
 kd[NAMax] = {  40.f ,  40.f ,  40.f ,  40.f ,  40.f ,  40.f ,   // 0 à 5
                40.f ,  40.f ,  40.f ,  40.f ,  40.f ,  40.f } ; // 6 à 11
	//static float com[3];
 int n, i;
	double t ; // s temps de l'échantillon

	for(n=2; n<NA+2; n++)
	{
				uLINKc[n].dq = (float)((qm[n-2] - uLINKc[n].q)/Te);
				uLINKc[n].q = qm[n-2];
	}

#if InertialLink
{ // Centrale
	for(n=0; n<9; n++)
	{
				uLINKc[1].R[n] = m[n];
	}
}
#endif

	David_ForwardKinematics(uLINKc,1);


 MatMulf( Statusc.FootCenter_R, uLINKc[7].R, FootR, 3, 3, 1) ;
 MatAddf( Statusc.FootCenter_R, Statusc.FootCenter_R, uLINKc[7].p, 3, 1) ;

 MatMulf( Statusc.FootCenter_L, uLINKc[13].R, FootL, 3, 3, 1) ;
 MatAddf( Statusc.FootCenter_L, Statusc.FootCenter_L, uLINKc[13].p, 3, 1) ;

 //David_CalcCoM(uLINKc,com);

	t = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps

 //David_OneFoot(qd, t-t0, &Statusc.desired_support, &Statusc.distribution_y);
 //David_OneFoot(dqd, t-t0-Dtime, &Statusc.desired_support, &Statusc.distribution_y);

	David_GravityExperiment(qd, t-t0, &Statusc.desired_support, &Statusc.distribution_y);
 David_GravityExperiment(dqd, t-t0-Te, &Statusc.desired_support, &Statusc.distribution_y);

	#if !CodeursCNA
	QcpStore ( qd ) ;
	#endif

 for(n=0; n<NA; n++)
 {
		{ // Test erreur de poursuite. Modifier suivant que c'est q ou m !!!!!!!!!!!!!!!!!!!  
			static const float
			#if 1 /////////////////////
			seuil[NAMax] = { SEP , SEP , SEP , SEP , SEP , SEP ,   // 0 à 5 rad
																				SEP , SEP , SEP , SEP , SEP , SEP } ; // 6 à 11
			#else
			seuil[NAMax] = { 0.125f , 0.125f , 0.125f , 0.125f , 0.125f , 0.125f ,   // 0 à 5
																				0.125f , 0.125f , 0.125f , 0.125f , 0.125f , 0.125f } ; // 6 à 11
			#endif
			//------------------
			if ( fabs( (double) qd[n]-qm[n] ) > seuil[i] ) Erreur( ErrPoursuite + i ) ;
		}

   uPD[n]=(float)(kd[n]*(((qd[n]-dqd[n])/Te)-uLINKc[n+2].dq)+kp[n]*(qd[n]-uLINKc[n+2].q));
 }


 //David_Stabilizator( uLINKc, &Statusc, uStab);


 David_Gravity( uLINKc, &Statusc, 1, fG, tG);
 for (n=0; n<NA; n++)
 {
   uG[n]=uLINKc[n+2].ug;
 }

 qmc=qu;
 for(n=0; n<NA; n++)
 {
			qmc[n] = uPD[n];
   //qmc[i] = uPD[n]+uG[n];
			//qmc[i] = uPD[n]+uG[n]+uStab[n];
 }

 #if Observateur
 for ( i = NA ; i-- ; )
 {
  ObsrvFloat(  2 + i , qd[i] ) ;
 } // for i
	for (i = NA ; i-- ; ) ObsrvFloat( 106 + i , uPD[i] ) ;
	for (i = NA ; i-- ; ) ObsrvFloat( 118 + i , uG[i] ) ;
 #endif //Observateur

}
#endif //David_couple

