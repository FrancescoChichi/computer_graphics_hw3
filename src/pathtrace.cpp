#include <thread>

#define YGL_IMAGEIO_IMPLEMENTATION 1
#include "yocto_gl.h"
#include "printData.h"

using namespace ygl;

using rng_t = ygl::rng_pcg32;

constexpr const auto ray_eps = 1e-4f;

struct point {
  const instance* ist = nullptr;
  vec3f x = zero3f; //position on shp
  vec3f n = zero3f; //normal on shape
  vec3f le = zero3f;//light emission
  vec3f o = zero3f; //outgoing direction
  vec3f kd = zero3f;//diffuse
  vec3f ks = zero3f;//specular
  float rs = 0.5;   //specular roughness

  vec3f kr(float cosw) const {
    return  ks + (vec3f{1, 1, 1} - ks) * pow(clamp(1.0f - cosw, 0.0f, 1.0f), 5.0f); }

  vec3f kt(float cosw) const{ return vec3f{1, 1, 1}-kr(cosw);}
  bool hit() const { return (bool)ist; }
  bool emission_only() const {
    return kd == vec3f{0, 0, 0} && ks == vec3f{0, 0, 0};
  }
};

/// Sample the camera for pixel i, j with image resolution res.
ray3f sample_camera(const camera* cam, int i, int j, int res, rng_t& rng) {


  auto u = (i+next_rand1f(rng))/res;
  auto v = 1 - (j+next_rand1f(rng))/res;

  float h = 2*tan(cam->yfov/2);
  float w = h*cam->aspect;

  auto height = res;
  auto width = (int)std::round(cam->aspect * res);


  auto o = vec3f{next_rand1f(rng) * cam->aperture, next_rand1f(rng) * cam->aperture, 0};
  auto q = vec3f{w * cam->focus * (u - 0.5f),
                 h * cam->focus * (v - 0.5f), -cam->focus};

  return ray3f(transform_point(cam->frame, o),
               transform_direction(cam->frame, normalize(q - o)));

/*
  vec3f ql = vec3f{(u-0.5f) * width, (v-0.5f) * height, -1};
  vec3f ol = vec3f{0, 0, 0};

  auto ray = transform_ray(cam->frame,
                           ray3f(zero3f,normalize(ql)));
  return ray;
  return {transform_point(cam->frame, ol), transform_direction(cam->frame, normalize(-ql - ol))};
*/

}

/// Evaluate the point proerties for a shape.
point eval_point(const instance* ist, int eid, const vec4f& euv, const vec3f& o) {

  auto p = point();

  p.ist=ist;
  p.o = o;

  p.x = eval_pos(ist->shp, eid, euv);;
  p.n = eval_norm(ist->shp, eid, euv);;
  p.kd = ist->shp->mat->kd;
  p.ks = ist->shp->mat->ks;
  p.rs = ist->shp->mat->rs;
  p.le = ist->shp->mat->ke;

  return p;
}

/// Evaluate the point proerties for an environment (only o and le).
point eval_point(const environment* env, const vec3f& o) {
  auto p = point();

  //p.le=env->ke;
  p.o= o;
  p.le=env->ke;
  /*
  // maerial
  auto ke = env->ke;
  if (env->ke_txt) {
    auto w = transform_direction(inverse(env->frame), -o);
    auto theta = (acos(clamp(w.y, (float)-1, (float)1)) / pif);
    auto phi = atan2(w.z, w.x) / (2 * pif);
    auto texcoord = vec2f{phi, theta};
    ke *= eval_texture(env->ke_txt, texcoord).xyz();
  }

  // create emission lobe
  if (ke != zero3f) { p.le =  ke; }
  //if (env->ke != zero3f) { p.le =  env->ke; }
*/
  // done
  return p;

}

/// Intersection the scene and return a point. Support both shape and
/// environments.
point intersect(
    const scene* scn, const vec3f& q, const vec3f& i, float tmax = flt_max) {

  ray3f ray = ray3f ({q,i,ray_eps,tmax});
  auto isec = ygl::intersect_ray(scn,ray,false);

  if(isec)
    return eval_point(scn->instances[isec.iid],isec.eid, isec.euv, -i);
  else
    return eval_point(scn->environments[0],-i);

}


/// Initialize the lights vector and compute the shape distribution with
/// `sample_XXX_cdf`. For the homework support only point and triangles.
void init_lights(scene* scn) {

  scn->lights.clear();

  for (auto lgt : scn->lights) delete lgt;
  scn->lights.clear();

  for (auto ist : scn->instances) {
    if (ist->shp->mat->ke == zero3f) continue;
    auto lgt = new light();
    lgt->ist = ist;
    auto shp = ist->shp;
    if (shp->elem_cdf.empty()) {
      if (!shp->points.empty()) {
        shp->elem_cdf = sample_points_cdf(shp->points.size());
      }
      else if (!shp->triangles.empty()) {
        shp->elem_cdf = sample_triangles_cdf(shp->triangles, shp->pos);
      }
    }
    scn->lights.push_back(lgt);
  }

  for (auto env : scn->environments) {
    if (env->ke == zero3f) continue;
    auto lgt = new light();
    lgt->env = env;
    scn->lights.push_back(lgt);
  }
  std::cerr<<"lights: "<<scn->lights.size()<<std::endl;

}

/// Pick one light at random and sample it with area sampling.
/// Returns the point on the light source evaluate through eval_point().
/// For sampling shapes, use the `sample_XXX()` functions.
/// For the homework support only point and triangles
/// If no lights are present, just return {}.
/// To use with MIS, fold the cosine at the light and r^2 into this funciton.
point sample_lights(const scene* scn, const point& pt, rng_t& rng) {

  /*if(scn->lights.empty())
    return {};
  else{

    //scn->lights[0]->ist->shp->mat->ke;
    return {};
  }*/

  auto lgt = scn->lights[0];
  auto rne = next_rand1f(rng);
  vec2f rn = {next_rand1f(rng),next_rand1f(rng)};

  if (lgt->ist) {
    auto shp = lgt->ist->shp;
    auto eid = 0;
    auto euv = zero4f;
    if (!shp->triangles.empty()) {
      std::tie(eid, (vec3f&)euv) =
          sample_triangles(shp->elem_cdf, rne, rn);
    } else if (!shp->points.empty()) {
      eid = sample_points(shp->elem_cdf, rne);
      euv = {1, 0, 0, 0};
    } else {
      assert(false);
    }
    auto lpt = eval_point(lgt->ist, eid, euv, zero3f);
    lpt.o = normalize(pt.x - lpt.x);
    return lpt;
  } else if (lgt->env) {
    auto z = -1 + 2 * rn.y;
    auto rr = sqrt(clamp(1 - z * z, (float)0, (float)1));
    auto phi = 2 * pif * rn.x;
    auto wo = vec3f{cos(phi) * rr, z, sin(phi) * rr};
    auto lpt = eval_point(lgt->env, wo);
    return lpt;
  } else {
    assert(false);
    return {};
  }
}

/// Compute the light sampling weight, which 1/pdf
float weight_lights(const scene* scn, const point& lpt, const point& pt) {
  //TODO weight_lights


  if (!lpt.hit()) return 0;
  // support only one lobe for now
  if(lpt.emission_only())//env
    return 4 * pif;
  else{
    auto d = dist(lpt.x, pt.x);
    return lpt.ist->shp->elem_cdf.back() / (d * d);
  }

}



// Evaluates the GGX distribution and geometric term
inline float eval_ggx(float rs, float ndh, float ndi, float ndo) {
  // evaluate GGX
  auto alpha2 = rs * rs;
  auto di = (ndh * ndh) * (alpha2 - 1) + 1;
  auto d = alpha2 / (pif * di * di);

  auto lambda_o = (-1 + sqrt(1 + alpha2 * (1 - ndo * ndo) / (ndo * ndo))) / 2;
  auto lambda_i = (-1 + sqrt(1 + alpha2 * (1 - ndi * ndi) / (ndi * ndi))) / 2;
  auto g = 1 / (1 + lambda_o + lambda_i);


  return d * g;
}



// Evaluates the GGX pdf
inline float pdf_ggx(float rs, float ndh) {
  auto cos2 = ndh * ndh;
  auto tan2 = (1 - cos2) / cos2;
  auto alpha2 = rs * rs;
  auto d = alpha2 / (pif * cos2 * cos2 * (alpha2 + tan2) * (alpha2 + tan2));
  return d;
}

// Sample the GGX distribution
inline vec3f sample_ggx(float rs, const vec2f& rn) {
  auto tan2 = rs * rs * rn.y / (1 - rn.y);
  auto rz = sqrt(1 / (tan2 + 1)), rr = sqrt(1 - rz * rz),
      rphi = 2 * pif * rn.x;
  // set to wh
  auto wh_local = vec3f{rr * cos(rphi), rr * sin(rphi), rz};
  return wh_local;
}


/// Evaluate the BSDF*cosine for a triangle surface. As BSDF use Kd/pi +
/// ks*D()*G()/4cos()cos(), using the GGX for D
vec3f eval_triangle_brdfcos(const point& pt, const vec3f& i) {///OK
  auto brdfcos = zero3f;

  const auto& o = pt.o;
  const auto& n = pt.n;
  auto h = normalize(o + i);

  // compute dot products
  auto ndo = dot(n, o), ndi = dot(n, i),
      ndh = clamp(dot(h, n), (float)-1, (float)1);
  auto odh = clamp(dot(o, h), 0.0f, 1.0f);

  // diffuse term
  if (ndi > 0 && ndo > 0) { brdfcos += pt.kd * ndi / pif; }

  // specular term
  if (ndi > 0 && ndo > 0 && ndh > 0) {
    // microfacet term
    auto dg = eval_ggx(pt.rs, ndh, ndi, ndo);

    // sum up
    brdfcos += pt.ks * ndi * dg / (4 * ndi * ndo);
  }

  // transmission hack
  if (o == -i) brdfcos += pt.kt(odh);

  return brdfcos;
}

/// Evaluate the BSDF*cosine for a line set. Left as example.
vec3f eval_line_brdfcos(const point& pt, const vec3f& i) {
  const auto& o = pt.o;
  const auto& n = pt.n;

  auto brdfcos = vec3f{0, 0, 0};

  auto h = normalize(o + i);
  auto ndo = dot(n, o), ndi = dot(n, i), ndh = dot(h, n);

  auto so = sqrt(clamp(1 - ndo * ndo, 0.0f, 1.0f)),
      si = sqrt(clamp(1 - ndi * ndi, 0.0f, 1.0f)),
      sh = sqrt(clamp(1 - ndh * ndh, 0.0f, 1.0f));

  if (pt.kd != vec3f{0, 0, 0}) {
    auto diff = pt.kd * (si / pif);
    brdfcos += diff;
  }

  if (pt.ks != vec3f{0, 0, 0}) {
    auto ns = 2 / (pt.rs * pt.rs) - 2;
    auto d = (ns + 2) * pow(sh, ns) / (2 + pif);
    auto spec = pt.ks * (si * d / (4.0f * si * so));
    brdfcos += spec;
  }

  return brdfcos;
}

/// Evaluate the BSDF*cosine for a point set. Left as example.
vec3f eval_point_brdfcos(const point& pt, const vec3f& i) {
  const auto& o = pt.o;

  auto ido = dot(o, i);
  return (pt.kd + pt.ks) * ((2 * ido + 1) / (2 * pif));
}

/// Evaluate the BSDF*cosine for a point.
vec3f eval_brdfcos(const point& pt, const vec3f& i) {
  if (pt.emission_only()) return {0, 0, 0};
  if (!pt.ist->shp->points.empty()) {
    return eval_point_brdfcos(pt, i);
  } else if (!pt.ist->shp->lines.empty()) {
    return eval_line_brdfcos(pt, i);
  } else if (!pt.ist->shp->triangles.empty()) {
    return eval_triangle_brdfcos(pt, i);
  } else {
    return {0, 0, 0};
  }
}


vec3f sample_spherical_dir(const point& pt, rng_t& rng) {
  auto rn = vec2f{next_rand1f(rng), next_rand1f(rng)};
  auto rz = rn.y, rr = sqrtf(1 - rz * rz), rphi = 2 * pif * rn.x;
  auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
  return transform_direction(make_frame3_fromz(pt.x, pt.n), wi_local);
}

float weight_spherical_dir() { return 1 / (4 * pif); }


/// Evaluate the BSDF*cosine as discussed in the slides
vec3f sample_triangle_brdfcos(const point& pt, rng_t& rng) {
  //TODO sample_triangle_brdfcos


  auto& wn = pt.n;
  auto& wo = pt.o;

  // skip if no component
  if (!pt.hit()) return zero3f;

  // probability of each lobe
  auto kdw = max_element_val(pt.kd), ksw = max_element_val(pt.ks),
      ktw = max_element_val(pt.kt(dot(wn,-wo)));
  auto kaw = kdw + ksw;// + ktw;
  kdw /= kaw;
  ksw /= kaw;
  ktw /= kaw + ktw;

  auto rnl = next_rand1f(rng);
  vec2f rn = {next_rand1f(rng),next_rand1f(rng)};

  // compute cosine
  auto ndo = dot(wn, wo);

  // check to make sure we are above the surface
  if (ndo <= 0) return zero3f;

  // sample according to diffuse
  if (rnl < kdw) {
    // sample wi with hemispherical cosine distribution
    auto rz = sqrtf(rn.y), rr = sqrtf(1 - rz * rz),
        rphi = 2 * pif * rn.x;
    // set to wi
    auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
    return transform_direction(make_frame3_fromz(pt.x, pt.n), wi_local);
  }
    // sample according to specular GGX
  else if (rnl < kdw + ksw) {
    // sample wh with ggx distribution
    auto wh_local = sample_ggx(pt.rs, rn);
    auto wh = transform_direction(make_frame3_fromz(pt.x, pt.n), wh_local);
    // compute wi
    return normalize(wh * 2.0f * dot(wo, wh) - wo);
  }
    // transmission hack
  else if (rnl < kdw + ksw + ktw) {
    // continue ray direction
    //return -wo;
  }

  return zero3f;
}

/// Comute the weight for BSDF sampling, i.e. 1 / pdf.
float weight_triangle_brdfcos(const point& pt, const vec3f& i) {
  //TODO weight_triangle_brdfcos

  // skip if no component
  if (!pt.hit()) return 0;



  // probability of each lobe
  auto kdw = max_element_val(pt.kd), ksw = max_element_val(pt.ks);

  auto kaw = kdw + ksw;
  kdw /= kaw;
  ksw /= kaw;

  // accumulate the probability over all lobes
  auto pdf = 0.0f;


  auto h = normalize(i + pt.o);

  // compute dot products
  auto ndo = dot(pt.n, pt.o),
      ndi = dot(pt.n, i),
      ndh = dot(pt.n, h);

  ///p(i)=n.i/pi

// compute wh

  // compute dot products

  // diffuse term (hemipherical cosine probability)
  if (ndo > 0 && ndi > 0) { pdf += kdw * ndi / pif; }

  // specular term (GGX)
  if (ndo > 0 && ndi > 0 && ndh > 0) {
    // probability proportional to d adjusted by wh projection
    auto d = pdf_ggx(pt.rs, ndh);
    auto hdo = dot(pt.o, h);
    pdf += ksw * d / (4 * hdo);
  }

  // transmission hack
  //if (i == -pt.o) pdf += ktw;

  return 1 / pdf;
}

/// Sample the BSDF*cosine
vec3f sample_brdfcos(const point& pt, rng_t& rng) {
  if (pt.emission_only()) return vec3f{0, 0, 0};
  if (!pt.ist->shp->points.empty()) {
    return sample_spherical_dir(pt, rng);
  } else if (!pt.ist->shp->lines.empty()) {
    return sample_spherical_dir(pt, rng);
  } else if (!pt.ist->shp->triangles.empty()) {
    return sample_triangle_brdfcos(pt, rng);
  } else {
    return {0, 0, 0};
  }
}

/// Weight for BSDF*cosine
float weight_brdfcos(const point& pt, const vec3f& i) {
  if (pt.emission_only()) return 0;
  if (!pt.ist->shp->points.empty()) {
    return weight_spherical_dir();
  } else if (!pt.ist->shp->lines.empty()) {
    return weight_spherical_dir();
  } else if (!pt.ist->shp->triangles.empty()) {
    return weight_triangle_brdfcos(pt, i);
  } else {
    return 0;
  }
}

/// Naive pathtracing called recurively. Hint: call reculsively with boucnes-1.
/// In this method, use hemispherical cosine sampling and only lambert BSDF.
vec3f estimate_li_naive(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {

  auto pt = intersect(scn, q, d);
  if(!pt.hit()) return pt.le;
  if(bounces <= 0) return pt.le;

  auto i = sample_brdfcos(pt, rng);
  auto li = estimate_li_naive(scn, pt.x,i,bounces-1,rng);

  auto lr = li * eval_brdfcos(pt,i) * weight_brdfcos(pt,i);

  return pt.le + lr;

}

/// Produce formulation of pathtracing that matches exactly eh above.
/// In this method, use hemispherical cosine sampling and only lambert BSDF.
vec3f estimate_li_product(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
  //TODO estimate_li_product

  auto pt = intersect(scn, q, d);
  auto li = pt.le;
  vec3f w = {1,1,1};

  for(auto bounce : range(bounces)) {
    if(!pt.hit()) break;
    auto i = sample_brdfcos(pt, rng);
    auto bpt = intersect(scn, pt.x, i);
    auto rrprob = 1.0f - ygl::min(ygl::max_element_val(pt.kd + pt.ks + pt.le), 0.95f);
    //rrprob = 1/rrprob;
    w *= eval_brdfcos(pt,i) * (rrprob*weight_brdfcos(pt,i)); // update w  //pr(pt.f)*p(i)
    li += w * bpt.le;          // accumulate li
    pt = bpt;                  // “recurse”
  }
  return li;
}

/// Pathtracing with direct+indirect and russian roulette
vec3f estimate_li_direct(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
  //TODO estimate_li_direct


  auto pt = intersect(scn, q, d);
  auto li = pt.le;
  vec3f w = {1,1,1};
  for(auto bounce : range(bounces)) {
    if(!pt.hit()) break;
    auto lpt = sample_lights(scn, pt, rng);
    if(!intersect(scn, pt.x, -lpt.o).hit())
      li += w*lpt.le * eval_brdfcos(pt,-lpt.o) * weight_lights(scn,lpt,pt);
  }
  return li;}

/// Pathtracing with direct+indirect, MIS and russian roulette
vec3f estimate_li_mis(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
  //TODO estimate_li_mis
  return {};
}

image4f pathtrace(const scene* scn, int resolution, int samples,
                  const std::string& integrator, int bounces, bool parallel) {
  auto cam = scn->cameras.front();
  auto img = image4f(
      (int)std::round(cam->aspect * resolution), resolution, {0, 0, 0, 0});

  auto estimate_li = estimate_li_naive;
  if (integrator == "naive") {
    estimate_li = estimate_li_naive;
  } else if (integrator == "product") {
    estimate_li = estimate_li_product;
  } else if (integrator == "direct") {
    estimate_li = estimate_li_direct;
  } else if (integrator == "mis") {
    estimate_li = estimate_li_mis;
  } else {
    throw std::runtime_error("bad integrator name");
  }

  if (!parallel) {
    for (auto j = 0; j < img.height(); j++) {
      for (auto i = 0; i < img.width(); i++) {
        auto rng = init_rng(0, (j * img.width() + i) * 2 + 1);
        img[{i, j}] = {0, 0, 0, 0};
        for (auto s = 0; s < samples; s++) {
          auto ray = sample_camera(cam, i, j, resolution, rng);
          auto li = estimate_li(scn, ray.o, ray.d, bounces, rng);
          if (!isfinite(li)) continue;
          img.at(i, j) += {li, 1};
        }
        img[{i, j}] /= (float)(samples);
      }
    }
  } else {
    auto nthreads = std::thread::hardware_concurrency();
    auto threads = std::vector<std::thread>();
    for (auto tid = 0; tid < nthreads; tid++) {
      threads.push_back(std::thread([=, &img]() {
        for (auto j = tid; j < img.height(); j += nthreads) {
          for (auto i = 0; i < img.width(); i++) {
            auto rng = init_rng(0, (j * img.width() + i) * 2 + 1);
            img[{i, j}] = {0, 0, 0, 0};
            for (auto s = 0; s < samples; s++) {
              auto ray =
                  sample_camera(cam, i, j, resolution, rng);
              img.at(i, j) +=
                  {estimate_li(scn, ray.o, ray.d, bounces, rng),
                   1};
            }
            img[{i, j}] /= (float)(samples);
          }
        }
      }));
    }
    for (auto& thread : threads) thread.join();
  }

  return img;
}

int main(int argc, char** argv) {
  // command line parsing
  auto parser = make_parser(argc, argv, "raytrace", "raytrace scene");
  auto parallel = parse_flag(parser, "--parallel", "-p", "runs in parallel");
  auto resolution = parse_opt<int>(
      parser, "--resolution", "-r", "vertical resolution", 720);
  auto samples =
      parse_opt<int>(parser, "--samples", "-s", "per-pixel samples", 1);
  auto bounces = parse_opt<int>(
      parser, "--bounces", "-b", "maximum number of bounces", 2);
  auto integrator =
      parse_opt<string>(parser, "--integrator", "-i", "estimation algorithm",
                        "direct", false, {"naive", "product", "direct", "mis"});
  auto imageout =
      parse_opt<string>(parser, "--output", "-o", "output image", "out.png");
  auto scenein =
      parse_arg<std::string>(parser, "scenein", "input scene", "scene.obj");
  if (should_exit(parser)) {
    std::cout << get_usage(parser);
    return 1;
  }

  // load scene
  log_info("loading scene " + scenein);
  auto scn = load_scene(scenein);

  // add missing data
  auto add_opts = add_elements_options::none();
  add_opts.smooth_normals = true;
  add_opts.pointline_radius = 0.001f;
  add_opts.shape_instances = true;
  add_opts.default_camera = true;
  add_opts.default_environment = true;
  add_elements(scn, add_opts);

  // create bvh
  log_info("creating bvh");
  build_bvh(scn);

  // init lights
  init_lights(scn);

  // raytrace
  log_info("tracing scene");
  auto hdr =
      pathtrace(scn, resolution, samples, integrator, bounces, parallel);
  // tonemap and save
  log_info("saving image " + imageout);
  auto ldr = tonemap_image(hdr, tonemap_type::srgb, 0, 2.2);
  save_image4b(imageout, ldr);
}
