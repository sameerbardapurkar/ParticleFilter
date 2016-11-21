#include <map.h>
#include <particle_state.h>

using namespace cv;
using namespace ps;

Map::Map(std::string filename, double max_range)
{	

  // Defining certain constants instead of reading from file
  size_x = 1300;
  size_y = 120;
  res = 1;

  grid_size = size_x*size_y;
  // initialize
  grid = Mat::zeros(size_y, size_x, CV_64FC1);
  grid_disp_ = Mat::zeros(size_y, size_x, CV_8UC3);
  // parse the map from dat file
  readMap(filename);

  rangemax = max_range/res;
  lidar_xoffset = 25;
  namedWindow( "Diag", WINDOW_AUTOSIZE );
  
}

Map::Map(Map* map) {
  size_x = map->size_x;
  size_y = map->size_y;
  res = map->res;
  grid_size = map->grid_size;
  grid = map->grid;
  grid_disp_ = map->grid_disp_;
  rangemax = map->rangemax;
  lidar_xoffset = map->lidar_xoffset;
  free_space_ = map->free_space_;
}
void Map::readMap(std::string file){

  std::ifstream fin(file);

  if (! fin.good())
  {
    printf("Bad File Path!\n");
    exit(EXIT_FAILURE);
  }

  std::string line, ch;
  int count = 0;
  double val;

  while (std::getline(fin, line))
  {

    count++;

    // skip first 7 lines
    if(count == 7)
      break;
  }

  // count = 0;
  // while ( fin >> val){
  //   int temprow = count / size_x;
  //   int tempcol = count % size_x;
  //   grid.at<double>((size_y - 1) - tempcol, temprow) = val;
  //   if(val == -1) {
  //     grid_disp_.at<Vec3b>((size_y - 1) - tempcol, temprow, 0) = Vec3b(0,0,0);
  //   }
  //   else {
  //     int b = (val)*255;
  //     int g = (val)*255;
  //     int r = (val)*255;
  //     grid_disp_.at<Vec3b>((size_y - 1) - tempcol, temprow, 0) = Vec3b(b,g,r);
  //   }
  //   //printf(" %f ", val);
  //   if (val == 1.0){
  //     double x = (size_y - 1) - tempcol;
  //     double y = temprow;
  //     for(double j = 1; j < res; j++)
  //       free_space_.push_back(std::make_pair((x*res) + j, (y*res) + j));
  //   }
  //   count++;
  // }

  count = 0;
  while ( fin >> val){
    int temprow = count / size_x;
    int tempcol = count % size_x;
    grid.at<double>(temprow, tempcol) = val;
    if(val == 0) {
      grid_disp_.at<Vec3b>(temprow, tempcol, 0) = Vec3b(0,0,0);
    }
    else {
      int b = (val)*255;
      int g = (val)*255;
      int r = (val)*255;
      grid_disp_.at<Vec3b>(temprow, tempcol, 0) = Vec3b(b,g,r);
    }
    //printf(" %f ", val);
    if (val == 1.0){
      double x = temprow;
      double y = tempcol;
      for(double j = 0; j < res; j++)
        free_space_.push_back(std::make_pair((x*res) + j, (y*res) + j));
    }
    count++;
  }
}

void Map::printMap(){

  cv::Size s = grid.size();

  printf("\nSize x : %d\n", s.width);
  printf("\nSize y : %d\n", s.height);
  printf("\nResolution : %d\n", res);
  printf("\nMap size : %d\n", grid_size);

  printf("\nMap : \n");

  for(int i = 0; i < s.height; i++){
    for(int j = 0; j < s.width; j++){

      std::cout << grid.at<double>(i,j) << " " ;
    }
    std::cout << std::endl;
  }
}

void Map::displayMap(){

  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", grid_disp_);                   // Show our image inside it.
  waitKey(0);   
  destroyWindow("Display Window");                                       // Wait for a keystroke in the window
}

void Map::visualizeParticles(vector<ParticleState>* particle_list, int color) {

  Mat grid_rgb = grid_disp_.clone();//(temp_grid.size(), CV_8UC3);
  //cvtColor(temp_grid, grid_rgb, CV_GRAY2RGB);

  Point pt;
  cv::Scalar c_color;
  if (color == 1) {
    c_color = cv::Scalar(0, 0, 255);
  }
  else {
    c_color = cv::Scalar(255, 0, 0);
  }
  for(std::vector<ParticleState>::iterator it = particle_list->begin(); it != particle_list->end(); ++it) {
    //std::cout << int(it->x()/res) << int(it->y()/res) << std::endl; 
    Eigen::Rotation2Dd t(it->theta());
    Eigen::Matrix2d rot_mat = t.toRotationMatrix();
    Eigen::Vector2d x_axis(200,0);
    x_axis = rot_mat*x_axis;
    Point x_tip((x_axis(1) + it->y())/res,(x_axis(0) + it->x())/res); 
    pt = Point(int(it->y()/res), int(it->x()/res)); // divided by res as one pixel in visualization = 10 units of distance
    circle(grid_rgb, pt, 2, c_color);
    //line(grid_rgb, pt, x_tip, c_color);
  }


  // Create a window for display.
  namedWindow( "Display window", WINDOW_AUTOSIZE );

  putText(grid_rgb, "10000 particles \n Time: 0 s", Point(0,0), 0, 1.0, cv::Scalar(255,255,255));
  imshow( "Display window", grid_rgb);                   // Show our image inside it.

  // sleep(0.1);

  waitKey(0);                                          // Wait for a keystroke in the window
  // destroyWindow( "Display window");
}

void Map::visualizeRobot(vector<ParticleState>* particle_list, int color, double time, int iter) {

  Mat grid_rgb = grid_disp_.clone();//(temp_grid.size(), CV_8UC3);
  //cvtColor(temp_grid, grid_rgb, CV_GRAY2RGB);

  Point pt;
  cv::Scalar c_color;
  //if (color == 1) {
    cv::Scalar r_color(0, 0, 255);
  //}
  //else {
    cv::Scalar p_color(255, 0, 0);
  //}
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  int size = particle_list->size();

  for(std::vector<ParticleState>::iterator it = particle_list->begin(); it != particle_list->end(); ++it) {
    //std::cout << int(it->x()/res) << int(it->y()/res) << std::endl; 
    x += it->x();
    y += it->y();
    theta += it->theta();
    Eigen::Rotation2Dd t(it->theta());
    Eigen::Matrix2d rot_mat = t.toRotationMatrix();
    Eigen::Vector2d x_axis(200,0);
    x_axis = rot_mat*x_axis;
    Point x_tip((x_axis(1) + it->y())/res,(x_axis(0) + it->x())/res); 
    pt = Point(int(it->y()/res), int(it->x()/res)); // divided by res as one pixel in visualization = 10 units of distance
    circle(grid_rgb, pt, 0.5, p_color);
    //line(grid_rgb, pt, x_tip, c_color);
  }
  x = x/size;
  y = y/size;
  theta = theta/size;

  Eigen::Rotation2Dd t(theta);
  Eigen::Matrix2d rot_mat = t.toRotationMatrix();
  Eigen::Vector2d x_axis(200,0);
  x_axis = rot_mat*x_axis;
  Point x_tip((x_axis(1) + y)/res,(x_axis(0) + x)/res); 
  pt = Point(int(y/res), int(x/res)); // divided by res as one pixel in visualization = 10 units of distance
//  if(time != )
  if(iter != -1) {
    circle(grid_rgb, pt, 6, r_color, 1, CV_AA);
    line(grid_rgb, pt, x_tip, r_color, 1, CV_AA);
  }
  stringstream ss;
  ss<<"Time: "<<time<<" s";
  string timestamp = ss.str();
  string particlecount = "Particles: 10,000";
  

  //
      int fontFace = FONT_HERSHEY_PLAIN;
      double fontScale = 1;
      int thickness = 1;

      //Mat img(600, 800, CV_8UC3, Scalar::all(0));

      int baseline=0;
      Size textSize1 = getTextSize(particlecount, fontFace,
                                  fontScale, thickness, &baseline);
      Size textSize2 = getTextSize(timestamp, fontFace,
                                  fontScale, thickness, &baseline);
      baseline += thickness;

      // center the text
      Point textOrg1(0,30);
      Point textOrg2(0,50);
      // draw the box
      /*rectangle(grid_rgb, textOrg + Point(0, baseline),
                textOrg + Point(textSize.width, -textSize.height),
                Scalar(255,255,255));*/
      // ... and the baseline first
      /*line(grid_rgb, textOrg + Point(0, thickness),
           textOrg + Point(textSize.width, thickness),
           Scalar(255, 255, 255));*/

      // then put the text itself
      putText(grid_rgb, particlecount, textOrg1, fontFace, fontScale,
              Scalar::all(255), thickness, 8);
      putText(grid_rgb, timestamp, textOrg2, fontFace, fontScale,
              Scalar::all(255), thickness, 8);
  //
  //putText(grid_rgb, text, Point(0,0), 0, 12.0, cv::Scalar(255,255,255));
  // Create a window for display.
  namedWindow( "Display window", WINDOW_AUTOSIZE );
  imshow( "Display window", grid_rgb);                   // Show our image inside it.

  // sleep(0.1);
  if(iter == -1) {
    waitKey(0);
  }
  else{
    waitKey(1);                                          // Wait for a keystroke in the window
  }
  // destroyWindow( "Display window");
}


void Map::visualizePoints(vector<pair<int,int>>* points_list) {

  Mat grid_rgb = grid_disp_.clone();//(temp_grid.size(), CV_8UC3);

  Point pt;
  cv::Scalar color(255, 0, 0);

  for(std::vector<pair<int,int>>::iterator it = points_list->begin(); it != points_list->end(); ++it) {
    pt = Point(it->first, it->second);
    circle(grid_rgb, pt, 1, color);
  }

  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", grid_rgb);                   // Show our image inside it.

  waitKey(1);                                          // Wait for a keystroke in the window
  destroyWindow( "Display window");
}

void Map::visualizeRayTrace(Mat& grid_rgb, ParticleState *particle, vector<pair<int,int>>* points_list, int r, int g, int b) {

  //Mat grid_rgb = grid_disp_.clone();//(temp_grid.size(), CV_8UC3);

  Point pt;
  cv::Scalar red(0, 0, 255);
  //int r;
  //int g;
  //int b;
  //int count = 0;
  //r = 255;
  int size = points_list->size();
  pt = Point(int(particle->y()/res), int(particle->x()/res)); // divided by res as one pixel in visualization = 10 units of distance
  circle(grid_rgb, pt, 5, red);
  //arrowedLine(grid_rgb, )
  cv::Scalar color(0, 0, 0);

  for(std::vector<pair<int,int>>::iterator it = points_list->begin(); it != points_list->end(); ++it) {
    pt = Point(it->second, it->first);
    /*if (r > 0 ) {
      r = r - (int)(2*count/size);
      b = b + (int)(2*count/size);
    }
    else {
      r = 0;
      b = b - (int)(2*count/size);
      g = g + (int)(2*count/size);
    }*/
    color = cv::Scalar(b,g,r);
    circle(grid_rgb, pt, 0.5, color);
    //count ++;
    //waitKey(0);
  }

  
}


vector<pair<int,int>> Map::interpolate(int x0, int y0, int x1, int y1) {

  vector<pair<int, int>> result;

  float dx = abs(x1 - x0);
  float dy = abs(y1 - y0);

  float dist = max(dx, dy);
  //printf("max dist=%f\n", dist);

  float t;
  int x, y;

  for (int step = 0; step <= dist ; step++) {
    if (dist == 0)
      t = 0.0;
    else t = step / dist;

    x = round(x0 + t*(x1-x0));
    y = round(y0 + t*(y1-y0));

    //printf("at step = %d points are x=%d y=%d t=%f", step,  x, y, t);
    //printf("grid value is =%f\n", grid.at<double>(x,y));

    if (grid.at<double>(x,y) < 1.0) {
      //// collided with obstacle
      //printf("collided");
      break;
    }

    result.push_back(make_pair(x,y));
  }

  return result;
}

void Map::visualizeIdealLidar(ParticleState p) {

  float theta;

  int x0 = (int) (p.x() ) / res, y0 = (int) (p.y() + lidar_xoffset) / res;
  double theta0 = p.theta();
  int x1, y1, tx, ty;
  //printf("x0=%d, y0=%d theta0=%f\n", x0, y0, theta0);

  vector<pair<int,int>> single_ray;
  vector<pair<int,int>> all_rays;
  //line(grid_rx0,)
  for (theta = 90; theta >= -90; theta--) {
    //{
    //theta = 0;

    x1 = rangemax * cos(theta * PI/180);
    y1 = rangemax * sin(theta * PI/180);


    Eigen::Vector2d point(x1, y1);
    Eigen::Rotation2Dd t(theta0);
    t.toRotationMatrix();

    Eigen::Vector2d new_point = t * point;

    tx = new_point(0) + x0;
    ty = new_point(1) + y0;

    single_ray = interpolate(x0, y0, tx, ty);
    all_rays.insert(all_rays.end(), single_ray.begin(), single_ray.end());

    //visualizeRayTrace(&p, &single_ray, 255, 0, 255);
  }

  
  }

//void interpolate(double x0, double y0, ParticleState& p, int index, Eigen::Vector2d tip,
//                     Eigen::Vector2d origin, double theta, Mat grid_p);

void Map::getIdealLidar(ParticleState* p) {
    //int num_threads = thread::hardware_concurrency();
    //cout<<"max threads are "<<num_threads<<endl;
    //ctpl::thread_pool pool(num_threads);
    //Mat grid_p = grid_disp_.clone();
    //float theta;
    //std::cout<<p.ranges()->size()<<std::endl;
    //p->ranges()->clear();
    for(int i = 0; i< p->ranges().size(); i++) {
     p->setRangeVal(i,res*rangemax);
    }
    double x0 = p->x();
    double y0 = p->y();
    Point query(y0/res, x0/res);
    double val = 1.0;
    if(query.x<0 || query.x >=grid.rows) {
      return;
    }
    if(query.y<0 || query.y >=grid.cols) {
      return;
    }
    
      val = grid.at<double>(query);
    
    /*catch(cv::Exception& e) {
      cout<<"Exception"<<endl;
      return;
    }*/

    if(val < 0.5) {
      //p->ranges() = std::vector<int>(p->ranges().size(), 0);
      p->weight(0.0); //Set its weight to 0
      return;
    }
    double theta = p->theta();
    std::vector<Eigen::Vector2d> rays = p->getRayTips();
    if(p->ranges().size() != 180) {
      cout<<"Error in laser ranges"<<endl;
    }
    /*for(int i = 0; i< p->ranges().size(); i++) {
      p->ranges().at(i) = 0;
    }*/
    //Delete Later
    /*cv::Scalar color(0,255,0);
    int b = 255;
    int g = 0;
    int r = 0;
    Point orig(y0/res, x0/res);
    color = cv::Scalar(b,g,r);
    circle(grid_p, orig, 5, color);*/
    //Delete Later*
    for(int i = 1; i < rays.size(); i++) {
      Eigen::Vector2d origin = rays[0];
      
      Eigen::Vector2d tip = rays[i];

      Eigen::Rotation2Dd t(theta);
      Eigen::Matrix2d rot_mat = t.toRotationMatrix();

      //Delete Later
      /*Eigen::Vector2d x_axis(rangemax*res,0);
      x_axis = rot_mat*x_axis;
      Point x_tip((x_axis(1) + y0)/res,(x_axis(0) + x0)/res);*/
      //Delete Later
      origin = rot_mat*origin;
      origin(0) = origin(0) + x0;
      origin(1) = origin(1) + y0;
      
      tip = rot_mat*tip;
      tip(0) = tip(0) + x0;
      tip(1) = tip(1) + y0;

      //Scale to image coords
      origin(0) = origin(0)/res;
      origin(1) = origin(1)/res;

      tip(0) = tip(0)/res;
      tip(1) = tip(1)/res;

      
      Point p1(origin(1), origin(0));
      Point p2(tip(1), tip(0));
      //Point hit;
      //pool.push(std::bind(&Map::interpolate1, this, p1, p2, p, i));
      //interpolate1(p1,p2,p,i);
      LineIterator it(grid, p1, p2);
      Point hit = p2;
      //cout<<"here"<<endl;
      for(int j = 0; j < it.count; j++, ++it) {
        double val = grid.at<double>(it.pos());
        if(val < 0.5) {
          hit = it.pos();
          break;
        }
      }
      /*if(i < 90) {
        color = cv::Scalar(0,0,255);
      }
      else {
        color = cv::Scalar(0,255,0);
      }*/
      //Delete Later
      /*circle(grid_p, hit, 2, color);
      //circle(grid_p, p2, 1, cv::Scalar(0,0,0));
      line(grid_p, p1, x_tip, cv::Scalar(255,0,0));*/
      //Delete Later
      double distance = res*(sqrt((hit.x-p1.x)*(hit.x-p1.x)
                            + (hit.y-p1.y)*(hit.y-p1.y)));
      //cout<<"Setting range as "<<distance<<endl;
      p->setRangeVal(i-1, distance);
      //cout<<"\t index is"<<index<<endl;

    }
    //pool.stop(true);
    //Delete Later
    /*namedWindow( "Diag", WINDOW_AUTOSIZE );
    imshow("Diag", grid_p);
    waitKey(0);
    destroyWindow("Diag");*/
    //Delete Later
}

void Map::getIdealLidarVis(ParticleState* p, data::lidar* lidar) {
    //int num_threads = thread::hardware_concurrency();
    //cout<<"max threads are "<<num_threads<<endl;
    //ctpl::thread_pool pool(num_threads);
    Mat grid_p = grid_disp_.clone();
    std::vector<int> measured_range = *lidar->ranges;
    //float theta;
    //std::cout<<p.ranges()->size()<<std::endl;
    //p->ranges()->clear();
    double x0 = p->x();
    double y0 = p->y();
    double theta = p->theta();
    bool setFlag = true;
    //cout<<theta<<endl;
    std::vector<Eigen::Vector2d> rays = p->getRayTips();
    if(p->ranges().size() != 180) {
      cout<<"Error in laser ranges"<<endl;
    }
    for(int i = 0; i< p->ranges().size(); i++) {
      p->setRangeVal(i,rangemax*res);
    }
    Point query(y0/res, x0/res);
    double val = 1.0;
    if(query.x<0 || query.x >=grid.rows) {
      return;
    }
    if(query.y<0 || query.y >=grid.cols) {
      return;
    }
    
      val = grid.at<double>(query);
    
    /*catch(cv::Exception& e) {
      cout<<"Exception"<<endl;
      return;
    }*/

    if(val < 1.0) {
      //p->ranges() = std::vector<int>(p->ranges().size(), 0);
      setFlag = false;
    }
    //Delete Later
    cv::Scalar color(0,255,0);
    int b = 255;
    int g = 0;
    int r = 0;
    Point orig(y0/res, x0/res);
    color = cv::Scalar(b,g,r);
    circle(grid_p, orig, 5, color);
    //Delete Later*
    for(int i = 1; i < rays.size(); i++) {
      Eigen::Vector2d origin = rays[0];
      
      Eigen::Vector2d tip = rays[i];

      Eigen::Rotation2Dd t(theta);
      Eigen::Matrix2d rot_mat = t.toRotationMatrix();

      //Delete Later
      Eigen::Vector2d x_axis(rangemax*res,0);
      x_axis = rot_mat*x_axis;
      double angle = -90 + (i-1)*(180/179);
      Eigen::Vector2d lidar_point(25 + measured_range[i-1]*cos(angle*PI/180), measured_range[i-1]*sin(angle*PI/180));
      lidar_point = rot_mat*lidar_point;
      Point x_tip((x_axis(1) + y0)/res,(x_axis(0) + x0)/res);
      Point lidar_tip((lidar_point(1) + y0)/res,(lidar_point(0) + x0)/res);
      //Delete Later
      origin = rot_mat*origin;
      origin(0) = origin(0) + x0;
      origin(1) = origin(1) + y0;
      
      tip = rot_mat*tip;
      tip(0) = tip(0) + x0;
      tip(1) = tip(1) + y0;

      //Scale to image coords
      origin(0) = origin(0)/res;
      origin(1) = origin(1)/res;

      tip(0) = tip(0)/res;
      tip(1) = tip(1)/res;

      
      Point p1(origin(1), origin(0));
      Point p2(tip(1), tip(0));
      //Point hit;
      //pool.push(std::bind(&Map::interpolate1, this, p1, p2, p, i));
      //interpolate1(p1,p2,p,i);
      LineIterator it(grid, p1, p2);
      Point hit = p2;
      if(setFlag) {
      for(int j = 0; j < it.count; j++, ++it) {
        double val = grid.at<double>(it.pos());
        if(val < 0.5) {
          hit = it.pos();
          break;
        }
      }
      }
      if(i < 90) {
        color = cv::Scalar(0,0,255);
      }
      else {
        color = cv::Scalar(0,255,0);
      }
      //Delete Later
      circle(grid_p, hit, 2, color);
      circle(grid_p, p2, 2, cv::Scalar(255,255,0));
      circle(grid_p, lidar_tip, 1, cv::Scalar(255,0,0));
      line(grid_p, p1, x_tip, cv::Scalar(255,0,0));
      //Delete Later
      double distance = res*(sqrt((hit.x-p1.x)*(hit.x-p1.x)
                            + (hit.y-p1.y)*(hit.y-p1.y)));
      double distance2 = res*(sqrt((p2.x-p1.x)*(p2.x-p1.x)
                            + (p2.y-p1.y)*(p2.y-p1.y)));
      int dist = (int) distance;
      //cout<<"Setting range as "<<dist<<endl;
      
        p->setRangeVal(i-1, dist);
      
      //cout<<"\t\t"<<measured_range[i-1]<<"\t"<<dist<<"\t"<<distance2<<endl;
      //cout<<"\t index is"<<index<<endl;

    }
    //pool.stop(true);
    //Delete Later
    //namedWindow( "Diag", WINDOW_AUTOSIZE );
    imshow("Diag", grid_p);
    waitKey(0);
    //destroyWindow("Diag");
    //Delete Later
}

void Map::interpolate1(Point p1, Point p2, ParticleState* p, int index) {
      LineIterator it(grid, p1, p2);
      Point hit;
      for(int j = 0; j < it.count; j++, ++it) {
        double val = grid.at<double>(it.pos());
        if(val < 1.0) {
          hit = it.pos();
          break;
        }
      }
      /*if(i < 90) {
        color = cv::Scalar(0,0,255);
      }
      else {
        color = cv::Scalar(0,255,0);
      }*/
      //Delete Later
      //circle(grid_p, hit, 2, color);
      //circle(grid_p, p2, 2, cv::Scalar(0,0,0));
      //line(grid_p, p1, x_tip, cv::Scalar(255,0,0));
      //Delete Later
      double distance = sqrt((hit.x-p1.x)*(hit.x-p1.x)
                            + (hit.y-p1.y)*(hit.y-p1.y));

      p->ranges().at(index-1) = distance;
      //cout<<"\t index is"<<index<<endl;
}

