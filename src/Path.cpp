#include <robospect_planner/Path.h>

Path::Path(){
  iCurrentWaypoint = iCurrentMagnet = -1;
  pthread_mutex_init(&mutexPath, NULL); //Initialization for WaypointRoutes' mutex
  bOptimized = false;
}

Path::~Path(){
  pthread_mutex_destroy(&mutexPath);
}

ReturnValue Path::AddWaypoint(Waypoint point){
  Waypoint aux;
  
  pthread_mutex_lock(&mutexPath);
  
  if(vPoints.size() > 0){
    aux = vPoints.back();
    // Only adds the waypoint if it's different from waypoint before
    if( (aux.dX != point.dX) || (aux.dY != point.dY) )
      vPoints.push_back(point);
  } else { // First point
    if(iCurrentWaypoint < 0){ //First point
      iCurrentWaypoint = 0;
    }
    
    vPoints.push_back(point);
  }
  
  pthread_mutex_unlock(&mutexPath); 
  return OK;
}

ReturnValue Path::AddWaypoint(vector <Waypoint> po){
  pthread_mutex_lock(&mutexPath);
  
  if(iCurrentWaypoint < 0){ //First point
    iCurrentWaypoint = 0;
  }
  for(int i = 0; i < po.size(); i++){
    vPoints.push_back(po[i]);
  }
  
  pthread_mutex_unlock(&mutexPath);
}

ReturnValue Path::CreateInterpolatedWaypoint(geometry_msgs::Pose2D pose){
  ReturnValue ret = OK;
  
  pthread_mutex_lock(&mutexPath);
  
  if(vPoints.size()> 0){
    vector <Waypoint> vAuxPoints;
    Waypoint new_point;
    new_point.dX = (vPoints[0].dX - pose.x) / 2.0;
    new_point.dY = (vPoints[0].dY - pose.y) / 2.0;
    new_point.dSpeed = vPoints[0].dSpeed;
    new_point.dA = vPoints[0].dA;
    vAuxPoints.push_back(new_point);
    for(int i = 0; i < vPoints.size(); i++)
      vAuxPoints.push_back(vPoints[i]);
    vPoints = vAuxPoints;
  }else{
    ret  = ERROR;
  }
  
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::AddMagnet(MagnetStruct magnet){
  pthread_mutex_lock(&mutexPath);

  if(iCurrentMagnet < 0){ //First point
    iCurrentMagnet = 0;
  }
  vMagnets.push_back(magnet);
  pthread_mutex_unlock(&mutexPath);
  
  return OK;
}

ReturnValue Path::AddMagnet(vector <MagnetStruct> po){
  pthread_mutex_lock(&mutexPath);
  
  if(iCurrentMagnet < 0){ //First point
    iCurrentMagnet = 0;
  }
  for(int i = 0; i < po.size(); i++){
    vMagnets.push_back(po[i]);
  }
  
  pthread_mutex_unlock(&mutexPath);
}

void Path::Clear(){
  pthread_mutex_lock(&mutexPath);
  iCurrentWaypoint = -1;
  iCurrentMagnet = -1;
  bOptimized = false;
  vPoints.clear();
  vMagnets.clear();
  pthread_mutex_unlock(&mutexPath);
}

ReturnValue Path::GetNextWaypoint(Waypoint *wp){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  
  if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < (vPoints.size() - 1)) ){
    *wp = vPoints[iCurrentWaypoint + 1];
    ret = OK;
  }
  
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::BackWaypoint(Waypoint *wp){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( vPoints.size() > 0){
    *wp = vPoints.back();
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::GetCurrentWaypoint(Waypoint *wp){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) ){
    *wp = vPoints[iCurrentWaypoint];
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::GetWaypoint(int index, Waypoint *wp){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( (index >= 0) && ( index< vPoints.size() ) ){
    *wp = vPoints[index];
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::SetCurrentWaypoint(int index){
  ReturnValue ret = ERROR;
  
  if(index < (vPoints.size() - 1)){
    pthread_mutex_lock(&mutexPath);
    iCurrentWaypoint = index;
    pthread_mutex_unlock(&mutexPath);
    ret = OK;
  }
  return ret;
}

ReturnValue Path::GetNextMagnet(MagnetStruct *mg){
  ReturnValue ret = ERROR;
  pthread_mutex_lock(&mutexPath);
  
  if( (iCurrentMagnet >= 0) && (iCurrentMagnet < (vMagnets.size() - 1)) ){
    *mg = vMagnets[iCurrentMagnet + 1];
    ret = OK;
  }
  
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::BackMagnet(MagnetStruct * mg){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( vMagnets.size() > 0){
    *mg = vMagnets.back();
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::GetCurrentMagnet( MagnetStruct * mg ){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( (iCurrentMagnet >= 0) && (iCurrentMagnet < vMagnets.size()) ){
    *mg = vMagnets[iCurrentMagnet];
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::GetPreviousMagnet( MagnetStruct * mg ){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size()) ){
    *mg = vMagnets[iCurrentMagnet-1];
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

ReturnValue Path::SetCurrentMagnet(int index){
  ReturnValue ret = ERROR;
  
  if(index < (vMagnets.size() - 1)){
    pthread_mutex_lock(&mutexPath);
    iCurrentMagnet = index;
    pthread_mutex_unlock(&mutexPath);
    ret = OK;
  }
  return ret;
}

ReturnValue Path::GetLastMagnet( MagnetStruct * mg ){
  ReturnValue ret = ERROR;
  
  pthread_mutex_lock(&mutexPath);
  if( (iCurrentMagnet > 0) && (iCurrentMagnet <= vMagnets.size() ) ){
    *mg = vMagnets[iCurrentMagnet - 1];
    ret = OK;
  }
  pthread_mutex_unlock(&mutexPath);
  
  return ret;
}

Waypoint Path::PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
  Waypoint aux;
  
  //B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
  //Bx(t)
  aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
  //By(t)
  aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;

  return aux;
}

double Path::DistForSpeed(double target_speed){
  if(target_speed > 1.0)
    return 2.0;
  else if(target_speed > 0.8)
    return 1.5;
  else return
	 1.0;
}

ReturnValue Path::Optimize(double distance){
  int i, j=0;
  int a, b, c;
  int x = 0, y = 1, speed = 2;
  double mod_ab, mod_bc;
  double dAngle;
  Waypoint ab, bc, ba;
  double K= 0.0;
  vector <Waypoint> new_points;
  Waypoint aux;
  double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0;
  Waypoint A, B, C;
  double Kt = 1.0 / BEZIER_CONTROL_POINTS;
  double dAuxSpeed = 0.0;
  double dMinDist = 0.0;	// Minica distancia a la que hay q frenar en funcion de la velocidad
  
  if(bOptimized){	//Already optimized
    return OK;
  }
  
  if((vPoints.size() < 2) || (distance <= 0.0)){	//Minimo 3 puntos en la ruta
    //printf("WaypointRoute::Optimize: Error: not enought points (%d)\n",Size());
    return ERROR;
  }
  pthread_mutex_lock(&mutexPath);
  //
  // Si solo hay dos puntos, interpolamos y creamos un punto intermedio
  if(vPoints.size() == 2){
    aux = vPoints[1];
    vPoints.push_back(aux); // Añadimos un punto al final y modificamos el del medio
    if((vPoints[0].dX - aux.dX) == 0.0){// Punto en el mismo eje X
      vPoints[1].dX = vPoints[0].dX;    // Mantenemos la coordenada en X
      vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
    }else if((vPoints[0].dY - aux.dY) == 0.0){ // Punto en el mismo eje Y
      vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
      vPoints[1].dY = vPoints[0].dY;    // Mantenemos la coordenada en Y
    }else{ // Punto en eje X, Y distinto
      vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; // La coordenada en X será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
      vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; // La coordenada en Y será igual a la del primer punto más la mitada de distancia entre el punto final e inicial
    }
  }
  
  new_points.push_back(vPoints[0]);
  new_points.push_back(vPoints[1]);
  
  for(i=2; i < vPoints.size(); i++){	// Primera pasada, añadimos puntos para los giros en curvas
    //cout << " Partiendo de punto " << i << endl;
    a = i-2;
    b = i-1;
    c = i;
    
    Ax = vPoints[a].dX;
    Ay = vPoints[a].dY;
    Bx = vPoints[b].dX;
    By = vPoints[b].dY;
    Cx = vPoints[c].dX;
    Cy = vPoints[c].dY;
    
    ab.dX = Bx - Ax;
    ab.dY = By - Ay;
    mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);
    
    bc.dX = Cx - Bx;
    bc.dY = Cy - By;
    mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
    
    dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
    //cout <<  i << " Angle =  "<< dAngle << endl;
    if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos para aproximar mjor la curva
      // siendo ba vector director de b->a y bc el vector director b->c
      // Calculamos un punto a una distancia 'd' desde 'b' hacia 'a' y otro punto desde 'b' hacia 'c'
      // Estos puntos serán los que se añadirán a la lista de waypoints para poder trazar una curva de bezier
      
      new_points.pop_back();
      
      // Calcula velocidad maxima en funcion del giro de la curva
      if(fabs(dAngle) >= (Pi/4)){
	dAuxSpeed = MAX_SPEED_LVL2;
      }else
	dAuxSpeed = MAX_SPEED_LVL1;
      //cout << "Aux speed = " << dAuxSpeed << ", Next speed =  " << vPoints[b].dSpeed << endl;
      // Si la velocidad en ese waypoint supera el máximo establecido para un giro así
      if(fabs(vPoints[b].dSpeed) > dAuxSpeed){
	
	if(vPoints[b].dSpeed < 0.0)	// Cambiamos sentido de avance
	  dAuxSpeed = -dAuxSpeed;
	
	dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));
	//cout << "Min dist = " << dMinDist  << endl;
	// Si el punto antes del giro esta a una distancia menor a la mínima, añadimos nuevo punto a un metro
	if( mod_ab > dMinDist){
	  //Lo creamos
	  ba.dX = -ab.dX;
	  ba.dY = -ab.dY;
	  K = dMinDist / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);
	  
	  aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
	  aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
	  aux.dSpeed = vPoints[b].dSpeed;
	  //cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
	  new_points.push_back(aux);
	}
	
	vPoints[b].dSpeed = dAuxSpeed;
	
      }
      // El primer waypoint no se modifica
      if(mod_ab > distance){ // si la distancia entre ab es MAYOR a la distancia del punto que pretendemos crear, creamos un punto intermedio
	//Lo creamos
	ba.dX = -ab.dX;
	ba.dY = -ab.dY;
	K = distance / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);
	
	aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
	aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
	aux.dSpeed = vPoints[b].dSpeed;
	//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
	new_points.push_back(aux);
	//j++;
      }
      
      new_points.push_back(vPoints[b]);
      
      if(mod_bc > distance){ // si la distancia entre ab es menor a la distancia del punto que pretendemos crear, lo dejamos como está
	//Lo creamos
	K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
	aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
	aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
	aux.dSpeed = vPoints[b].dSpeed;
	//j++;
	//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
	new_points.push_back(aux);
      }
      
      // Creamos punto para después del giro
      if(dMinDist > 0.0) {
	if(mod_bc > 1.0){	// si la distancia al punto C es mayor que 1 metro, después del giro
	  // Creamos un nuevo punto
	  //Lo creamos
	  K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
	  aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
	  aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
	  aux.dSpeed = vPoints[b].dSpeed;
	  new_points.push_back(aux);
	}else{
	  // Si no, establecemos una velocidad máxima
	  vPoints[c].dSpeed = vPoints[b].dSpeed;
	}
      }
      
      
      new_points.push_back(vPoints[c]);
      //j++;
    }else{	//Se queda como está
      
      new_points.push_back(vPoints[c]);
      
    }
  }
  
  // Borramos antiguos waypoints e insertamos los nuevos
  vPoints.clear();
  
  // BEZIER
  vPoints.push_back(new_points[0]);
  vPoints.push_back(new_points[1]);
  for(i=2; i < new_points.size(); i++){	// Segunda pasada, aproximamos los giros a curvas de Bezier
    a = i-2;
    b = i-1;
    c = i;
    
    Ax = new_points[a].dX;
    Ay = new_points[a].dY;
    Bx = new_points[b].dX;
    By = new_points[b].dY;
    Cx = new_points[c].dX;
    Cy = new_points[c].dY;
    
    ab.dX = Bx - Ax;
    ab.dY = By - Ay;
    mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);
    
    bc.dX = Cx - Bx;
    bc.dY = Cy - By;
    mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
    
    dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
    
    if(fabs(dAngle) >= MIN_ANGLE_BEZIER){ // en caso del angulo que forman los segmentos sea menor, entonces generaremos puntos, siguiendo una curva de Bezier, para aproximar mjor la curva
      // siendo ba vector director de b->a y bc el vector director b->c
      
      Waypoint aux_wp;
      double t, aux_speed;
      
      A = new_points[a];
      B = new_points[b];
      C = new_points[c];
      
      aux_speed = new_points[b].dSpeed; //takes speed of the waypoint in the middle
      vPoints.pop_back();			// Eliminamos el waypoint del medio. El primer waypoint no se modifica
      
      for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
	t = (double) j * Kt;
	aux_wp = PosOnQuadraticBezier(A, B, C,  t);
	aux_wp.dSpeed = aux_speed;
	vPoints.push_back(aux_wp);
	//	std::cout << "\tWaypointRoute::Optimize: (Bezier) Waypoint,X= " << aux_wp.dX << " Y= " << aux_wp.dY
	//				<< " size= " << (int)points.size() << endl;
      }
    }else{	//Se queda como está
      
      vPoints.push_back(new_points[c]);
    }
  }
  
  iCurrentWaypoint = 0;
  
  //	for(int i = 0; i < new_points.size(); i++){
  //	points.push_back(new_points[i]);
  //	}
  
  pthread_mutex_unlock(&mutexPath);
  
  bOptimized  = true;
  new_points.clear();
  
  return OK;
}

void Path::Print(){
  cout << "Path::Print: Printing all the waypoints..." << endl;
  if(vPoints.size() > 0){
    for(int i = 0; i < vPoints.size(); i++){
      cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
    }
  }else
    cout << "Path::Print: No waypoints..." << endl;
}
