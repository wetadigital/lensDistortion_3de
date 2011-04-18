
/************************************************************************
*
* Copyright (c) 2011, Weta Digital Limited
*
* All rights reserved.
*
*    Redistribution and use in source and binary forms, with or
*    without modification, are permitted provided that the
*    following conditions are met:
*
*     * Redistributions of source code must retain the above
*       copyright notice, this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following disclaimer
*       in the documentation and/or other materials provided with the distribution.
*
*     * Neither the name of the Weta Digital nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
*
*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*    POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************/

#include "DDImage/Iop.h"
#include "DDImage/Row.h"
#include "DDImage/Tile.h"
#include "DDImage/Knobs.h"
#include <list>
#include <sys/types.h>
#include <dirent.h>
#include <ldpk/ldpk_plugin_loader.h>
#include <ctype.h>
#include "DDImage/Thread.h"
#include "DDImage/Filter.h"
#include "DDImage/Pixel.h"

using namespace DD::Image;

  using std::list;
  using std::string;
  using std::cerr;
  using std::endl;
  using std::pair;
  using std::max;
  using std::min;
  using std::vector;
  using ldpk::plugin_loader;
  
namespace
{
  ///
  /// paraminfo stores info about a lens model parameter
  ///
  struct paraminfo{
    string name_ldpk; ///< names with spaces are permitted in ldpk -  use that for the label
    string name_nuke; ///< strip out spaces for nuke internal
    bool builtin;     ///< true if the parameter is updated by a call to 
                      ///  set_builtin_parameter_value rather than setParameterValue
  };
}



///
/// main node - a straight Iop. We handle warping internally
///
class LensDistortion_3de : public Iop
{
   const char * _node_name; ///< the name of this node - set on load
   char * _node_help;       ///< help string to return - got from the plugin
   
   plugin_loader * pl;      ///< plugin loaders for threads - we have one of these per thread
   Lock * lock;             ///< locks for each pl object
   int array_size;          ///< size of pl and lock arrays
   
   int k_direction;         ///< distort or undistort?
   
   Filter filter;///< filter knob

   ///
   /// double parameters required by the lens model. Currently string parameters are not supported
   /// this can be added trivially
   ///
   list< pair <paraminfo , double > > k_doubles; 
   
   public:
     
    LensDistortion_3de(Node *,int model); ///< construct an implementation of model.names[i]
    
    void _validate(bool);
    void _request(int,int,int,int,ChannelMask,int);
    void engine(int y,int x,int r,ChannelMask,Row &);
    virtual void knobs(Knob_Callback);
    const char * Class() const { return _node_name;}
    const char * node_help() const { return _node_help;}
    
    
    /// return a bounding box 
    Box bounds(bool direction,const Box &in,double width,double height);
    
   ~LensDistortion_3de() { delete [] pl;delete [] lock;}
};


//
/// an array of construction functions - 
/// each one of these constructs an Op implementing a different lens model
/// (this is working around the fact that Nuke doesn't tell us
///  the type of the Op it wants us to construct, so we have to make a constructor for each type)
//

template<int T> Op * c(Node * node) { return new LensDistortion_3de(node,T);}

Op * (* functions[])(Node *node) = { &c<0> , &c<1> , &c<2> , &c<3> , &c<4> , &c<5> , &c<6> , &c<7> , &c<8> , &c<9> , &c<10> , &c<11> , &c<12>};



namespace
{
   ///
   /// on load, the 'names' object of this type parses LENSMODEL_PATH looking for .so files
   /// constructs an array of plugin names, and informs Nuke about them by creating
   /// description objects
   ///
  
class StaticInfo{
  
 public:
   
   vector<const char * > names;
   string plg_path;
   StaticInfo()
   {
      //
      // parse the directory
      //

    static const char * fallback_path=LENSMODEL_PATH;

    const char * var=getenv("LENSMODEL_PATH");
   
    
    if(var)
    {
      plg_path=var;
    }else{
      plg_path=fallback_path; //env var not set - fall back to compiled in path
    }

    //
    // scan directory for any .so files
    //
    DIR * dir = opendir(plg_path.c_str());
    if(dir)
    {
      struct dirent * ent;
      while((ent=readdir(dir))!=NULL)
      {
        char * ext = strstr(ent->d_name,".so");
        if(ext)
        {
          string s(ent->d_name,ext-ent->d_name);
          
          char * name = new char [s.length()+1];
          strcpy(name,s.c_str());

          names.push_back(name);
          

          //
          // node name is plugin name
          // make a description out of name, using one of the functions in our table
          // as the constructor function
          //
          new Iop::Description(name,functions[names.size()-1]);
          
          
          //
          // more than 12? probably a bad path, so bail
          //
          if(names.size()>12)
          {
            std::cerr << "warning: maximum number of lens distortions plugins reached. Remove unwanted plugins from " << plg_path << " or recompile LensDistortion_3de" << std::endl;
            return;
          }
        }        
      }
    }
   }
   };

   StaticInfo models;

}



LensDistortion_3de::LensDistortion_3de(Node * n,int num) : Iop(n) , _node_name(models.names[num]) , k_direction(0)
{
  //
  // work out how many parallel plugins we need (we need one per thread, since they aren't threadsafe)
  //
  array_size = std::max(16u,Thread::numCPUs+Thread::numThreads+1);
  
  
  pl = new plugin_loader[array_size];
  lock = new Lock[array_size];
      
  // load the plugin into each thread
  for(int i = 0 ; i < array_size;i++)
  {
    pl[i].open_plugin(models.plg_path+"/"+_node_name+".so");
  }
  
  
  //
  // nice sensible name for the help
  //
  char model_name[100];

  pl[0].get_model()->getModelName(model_name);
  std::string h = "3DEqualiser lens distortion model name: "+std::string(model_name);
  _node_help = new char[h.length()+1];
  strcpy(_node_help,h.c_str());

}

static const char * const directions[] = {"distort","undistort",0};

void LensDistortion_3de::knobs(Knob_Callback f)
{
  //direction knob 
  Enumeration_knob(f,&k_direction,directions,"direction");
  
  filter.knobs(f);
  
  // the library hard-codes the number of builtin parameters and their names (effectively) so we'll do the same.  
  
  //builtin parameters first, then others

  pair<paraminfo,double> d;
  d.first.name_ldpk="tde4_focal_length_cm";
  d.first.name_nuke="tde4_focal_length_cm";
  d.first.builtin=true;
  d.second=10.0;
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());
  
  d.first.name_ldpk="tde4_filmback_width_cm";
  d.first.name_nuke="tde4_filmback_width_cm";
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());

  d.first.name_ldpk="tde4_filmback_height_cm";
  d.first.name_nuke="tde4_filmback_height_cm";
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());

  d.first.name_ldpk="tde4_lens_center_offset_x_cm";
  d.first.name_nuke="tde4_lens_center_offset_x_cm";
  d.second=0.0;
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());

  d.first.name_ldpk="tde4_lens_center_offset_y_cm";
  d.first.name_nuke="tde4_lens_center_offset_y_cm";
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());


  d.first.name_ldpk="tde4_pixel_aspect";
  d.first.name_nuke="tde4_pixel_aspect";
  k_doubles.push_back(d);
  Double_knob(f,&k_doubles.back().second,d.first.name_nuke.c_str(),d.first.name_ldpk.c_str());
 
  
  //
  // now query the model and make a knob for each of its parameters (only doubles supported)
  //
 int params;
 pl[0].get_model()->getNumParameters(params);
 for(int i = 0 ; i < params ; i++)
 {
      char name[4096];
      pl[0].get_model()->getParameterName(i,name);
      tde4_ldp_ptype ptype;
      pl[0].get_model()->getParameterType(name,ptype);
      switch(ptype)
      {
        // no standard models use these types, so we can't test them!
        case  TDE4_LDP_STRING : case TDE4_LDP_INT : case TDE4_LDP_FILE : case  TDE4_LDP_TOGGLE : 
           std::cerr << "plugin " << _node_name << " has an unsupported parameter type. This needs adding to lensDistoriton_3de\n";
           break;
           
        case  TDE4_LDP_DOUBLE : case TDE4_LDP_ADJUSTABLE_DOUBLE :
        {
            //
            // generate clean name for parameter
            //
            pair<paraminfo,double> d;
            d.first.name_ldpk=string(name);
            d.first.name_nuke=string(name);
            //change nasty names to underscores for nuke names
            for(size_t i = 0;i<d.first.name_nuke.length();i++)
            {
              if(!isalnum(d.first.name_nuke.at(i)))
                d.first.name_nuke[i]='_';
            }
            pl[0].get_model()->getParameterDefaultValue(name,d.second);
            k_doubles.push_back(d);
            
            //
            // declare as a Nuke knob, with the appropriate range
            //
            Double_knob(f,&k_doubles.back().second,
                        k_doubles.back().first.name_nuke.c_str(),
                        k_doubles.back().first.name_ldpk.c_str());
            double near,far;
         
            if(pl[0].get_model()->getParameterRange(name,near,far))
            {
              SetRange(f,near,far);
            }
         }
      } 
  }// next parameter type

}

//
/// \brief compute the distorted/undistorted bounding box of input, given width and height of image
///
/// only thorough way of doing this appears to be to walk around the edge
/// even doing that is a bit iffy, but there's nothing better we can do
/// since we assume nothing about the image
///
/// (would be nice if ldpk forced its lens models to implement this function internally!)
//
Box LensDistortion_3de::bounds(bool direction,const Box & input,double width,double height)
{
  //top and bottom
  
  Box out;
  
  bool init = false;
  for(int i = input.x();i<input.r();i++)
  {
    for(int pass=0;pass<2;pass++)
    {
      double x = (i+0.5)/width;
      double y = ((pass==0 ? input.y() : input.t())+0.5) / height;
      
      double x_out;
      double y_out;
      
      if(direction)
      {
        pl[0].get_model()->distort(x,y,x_out,y_out);
      }else{
        pl[0].get_model()->undistort(x,y,x_out,y_out);
      }
      
      if(x_out-x_out==0 && y_out-y_out==0)
      {
      
        x_out=(x_out*width)-0.5;
        y_out=(y_out*height)-0.5;
        if(!init)
        {
          out.x(int(floor(x_out)));
          out.y(int(floor(y_out)));
          out.r(int(ceil(x_out)));
          out.t(int(ceil(y_out)));
          init=true;
        }else{
          out.merge(int(floor(x_out)),int(floor(y_out)));
        }
      }
    }
  }
  
  //left and right
  
  for(int j = input.y();j<input.t();j++)
  {
    for(int pass=0;pass<2;pass++)
    {
      double x = ((pass==0 ? input.x() : input.r())+0.5) / width;
      double y = (j+0.5)/height;
      
      double x_out;
      double y_out;
      
      if(direction)
      {
        pl[0].get_model()->distort(x,y,x_out,y_out);
      }else{
        pl[0].get_model()->undistort(x,y,x_out,y_out);
      }
      
      if(x_out-x_out==0 && y_out-y_out==0)
      {
        x_out=(x_out*width)-0.5;
        y_out=(y_out*height)-0.5;
        out.merge(int(floor(x_out)),int(floor(y_out)));
      }
    }
  }
  
  return out;
  
}

void LensDistortion_3de::_validate(bool for_real)
{
  copy_info();
 
  //
  // update model parameters for each of our plugins
  //
    
  for(int i = 0 ; i < array_size;i++)
  {
    for(list<pair<paraminfo,double> >::const_iterator j  = k_doubles.begin() ; j!=k_doubles.end() ; j++)
    {
      pl[i].get_model()->setParameterValue(j->first.name_ldpk.c_str(),j->second);
    } 
    pl[i].get_model()->initializeParameters();
  }

  filter.initialize();
  
  //
  // set the output bounding box according to the bounds
  // since this is downwards not upwards, we have to flip the direction
  //
  
  Box box = bounds(1-k_direction,info_,format().width(),format().height());
  info_.set(box);
}


//
/// given an output bounding box, compute the input bounding box and request from image
//
void LensDistortion_3de::_request(int x,int y,int r,int t,ChannelMask channels,int count)
{
  if(!input(0)) return;
  Box box(x,y,r,t);
  box= bounds(k_direction,box,format().width(),format().height());

  box.x(max(input0().info().x(),box.x()));
  box.y(max(input0().info().y(),box.y()));
  
  box.r(min(input0().info().r(),box.r()));
  box.t(min(input0().info().t(),box.t()));
  
  input0().request(box.x(),box.y(),box.r(),box.t(),channels,count);
}

void LensDistortion_3de::engine(int y,int x,int r,ChannelMask channels,Row & outrow)
{
  if(!input(0)) return;

  double h = format().height();
  double w = format().width();

  double y_s = (y+0.0)/h;
//  double y_s = (y+0.5)/h;

  ///
  /// warp all rows - track bounding box of request
  ///
  
  vector<pair<double,double> > distort(r-x);

  double x_min = 1000000;
  double x_max = -x_min;
  double y_min = 1000000;
  double y_max = -y_min;

  //work out which of the array of plugins we should use
  int lnum=0;
  const Thread::ThreadInfo * f = Thread::thisThread();
  if(f)
  {
    lnum=f->index+1;
  }
  
  //safety fallback - use the first
  if(lnum>array_size) lnum=0;
  
  
  const Info & info = input0().info();
  
  
  //
  // threadsafe read of distortion data
  //
  {
    
    //
    // hopefully we should never contend on this lock: each thread should be running
    // on a separate plugin, so have a separate lock
    //
    Guard l(lock[lnum]); 
  
    // distort each pixel on row - store result in array, and track bounding box
    for(int i = x;i<r;i++)
    {
      double i_s=(i+0.0)/w;

      double x_o,y_o;
      if(k_direction)
      {
        pl[lnum].get_model()->distort(i_s,y_s, x_o,y_o);
      }else{
        pl[lnum].get_model()->undistort(i_s,y_s, x_o,y_o);
      }
      //'sample' takes pixel-centre positions, so no adjustment required
      x_o*=w;
      y_o*=h;
      if(info.x()>x_o) x_o = info.x();
      if(info.y()>y_o) y_o = info.y();
      if(info.r()-1.0<x_o) x_o = info.r()-1;
      if(info.t()-1.0<y_o) y_o = info.t()-1;
      
      distort[i-x].first=x_o;
      distort[i-x].second=y_o;

      if(x_o<x_min) x_min=x_o;
      if(x_o>x_max) x_max=x_o;
      if(y_o<y_min) y_min=y_o;
      if(y_o>y_max) y_max=y_o;

    }
  }
  
  //
  // now we know which pixels we'll need, request them!
  //
  y_max++;
  x_max++;
  
  Pixel out(channels);

  // lock the tile into the cache
  Tile t(input0(),int(floor(x_min)),int(floor(y_min)),int(ceil(x_max)),int(ceil(y_max)),channels);

  //
  // loop over our array of precomputed points, and ask nuke to perform a filtered lookup for us
  //
  for(int i = x;i<r;i++)
  {
    if(aborted()) break;
    input0().sample(distort[i-x].first+0.5,distort[i-x].second+0.5,1.0,1.0,&filter,out);
    foreach (z,channels)
    {
      outrow.writable(z)[i]=out[z];
    }
  }
  

}

