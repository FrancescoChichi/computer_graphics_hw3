#include <thread>

#define YGL_IMAGEIO_IMPLEMENTATION 1
#include "yocto_gl.h"

using namespace ygl;
using rng_t = ygl::rng_pcg32;

constexpr const auto ray_eps = 1e-4f;

struct point {
  const instance* ist = nullptr;
  vec3f x = zero3f; //origin
  vec3f n = zero3f; //z
  vec3f le = zero3f;
  vec3f o = zero3f; //direction
  vec3f kd = zero3f;
  vec3f ks = zero3f;
  float rs = 0.5;

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

  // instance
  p.ist = ist;

  // direction
  p.o = o;

  // shortcuts
  auto shp = ist->shp;
  auto mat = ist->shp->mat;

  // compute points and weights
  auto pos = eval_pos(ist->shp, eid, euv);
  auto norm = eval_norm(ist->shp, eid, euv);
  auto texcoord = eval_texcoord(ist->shp, eid, euv);
  auto color = eval_color(ist->shp, eid, euv);

  // handle normal map
  if (mat->norm_txt) {
    auto tangsp = eval_tangsp(ist->shp, eid, euv);
    auto txt = eval_texture(mat->norm_txt, texcoord, false).xyz() * 2.0f -
               vec3f{1, 1, 1};
    auto ntxt = normalize(vec3f{txt.x, -txt.y, txt.z});
    auto frame =
        make_frame3_fromzx({0, 0, 0}, norm, {tangsp.x, tangsp.y, tangsp.z});
    frame.y *= tangsp.w;
    norm = transform_direction(frame, ntxt);
  }

  // correct for double sided
  if (mat->double_sided && dot(norm, o) < 0) norm = -norm;

  // creating frame
  p.x = make_frame3_fromz(transform_point(ist->frame, pos),
                               transform_direction(ist->frame, norm)).o;

  // handle color
  auto kx_scale = vec4f{1, 1, 1, 1};
  if (!shp->color.empty()) kx_scale *= color;

  // handle occlusion
  if (mat->occ_txt)
    kx_scale.xyz() *= eval_texture(mat->occ_txt, texcoord).xyz();

  // sample emission
  auto ke = mat->ke * kx_scale.xyz();

  // sample reflectance
  auto kd = zero4f, ks = zero4f, kt = zero4f;
  switch (mat->mtype) {
    case material_type::specular_roughness: {
      kd = vec4f{mat->kd, mat->op} * kx_scale *
           eval_texture(mat->kd_txt, texcoord);
      ks = vec4f{mat->ks, mat->rs} * vec4f{kx_scale.xyz(), 1} *
           eval_texture(mat->ks_txt, texcoord);
    }
      break;
    case material_type::metallic_roughness: {
      auto kb = vec4f{mat->kd, mat->op} * kx_scale *
                eval_texture(mat->kd_txt, texcoord);
      auto km = vec2f{mat->ks.x, mat->rs};
      if (mat->ks_txt) {
        auto ks_txt = eval_texture(mat->ks_txt, texcoord);
        km.x *= ks_txt.y;
        km.y *= ks_txt.z;
      }
      kd = vec4f{kb.xyz() * (1 - km.x), kb.w};
      ks =
          vec4f{kb.xyz() * km.x + vec3f{0.04f, 0.04f, 0.04f} * (1 - km.x),
                km.y};
    }
      break;
    case material_type::specular_glossiness: {
      kd = vec4f{mat->kd, mat->op} * kx_scale *
           eval_texture(mat->kd_txt, texcoord);
      ks = vec4f{mat->ks, mat->rs} * vec4f{kx_scale.xyz(), 1} *
           eval_texture(mat->ks_txt, texcoord);
      ks.w = 1 - ks.w;  // glossiness -> roughness
    }
      break;
  }

  // set up final values
  p.le = ke * kd.w;
  p.kd = kd.xyz() * kd.w;
  p.ks =
      (ks.xyz() != zero3f && ks.w < 0.9999f) ? ks.xyz() * kd.w : zero3f;
  p.rs = (ks.xyz() != zero3f && ks.w < 0.9999f) ? ks.w * ks.w : 0;



  // done
  return p;


  /*p.ist=ist;
  p.o = o;
  p.kd = ist->shp->mat->kd;
  p.ks = ist->shp->mat->ks;
  p.rs = ist->shp->mat->rs;
  p.le = ist->shp->mat->ke;

  return p;*/
}

/// Evaluate the point proerties for an environment (only o and le).
point eval_point(const environment* env, const vec3f& o) {
  auto p = point();

  //p.le=env->ke;
  p.o= o;

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

  // done
  return p;


  /***
   * // set shape data
    auto pt = point();

    // env
    pt.env = env;

    // direction
    pt.wo = wo;

    // maerial
    auto ke = env->ke;
    if (env->ke_txt) {
        auto w = transform_direction(inverse(env->frame), -wo);
        auto theta = (acos(clamp(w.y, (float)-1, (float)1)) / pif);
        auto phi = atan2(w.z, w.x) / (2 * pif);
        auto texcoord = vec2f{phi, theta};
        ke *= eval_texture(env->ke_txt, texcoord).xyz();
    }

    // create emission lobe
    if (ke != zero3f) { pt.em = {emission_type::env, ke}; }

    // done
    return pt;
   */
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
  update_lights(scn,false);

  std::cerr<<"lights: "<<scn->lights.size()<<std::endl;

}

/// Pick one light at random and sample it with area sampling.
/// Returns the point on the light source evaluate through eval_point().
/// For sampling shapes, use the `sample_XXX()` functions.
/// For the homework support only point and triangles
/// If no lights are present, just return {}.
/// To use with MIS, fold the cosine at the light and r^2 into this funciton.
point sample_lights(const scene* scn, const point& pt, rng_t& rng) {
  if(scn->lights.empty())
    return {};
  else{

    scn->lights[0]->ist->shp->mat->ke;
    return {};
  }
}

/// Compute the light sampling weight, which 1/pdf
float weight_lights(const scene* scn, const point& lpt, const point& pt) {
  //TODO weight_lights
  return {};
}



// Schlick approximation of Fresnel term
inline vec3f eval_fresnel_schlick(const vec3f& ks, float cosw) {
  return ks +
         (vec3f{1, 1, 1} - ks) * pow(clamp(1.0f - cosw, 0.0f, 1.0f), 5.0f);
}

// Schlick approximation of Fresnel term weighted by roughness.
// This is a hack, but works better than not doing it.
inline vec3f eval_fresnel_schlick(const vec3f& ks, float cosw, float rs) {
  auto fks = eval_fresnel_schlick(ks, cosw);
  return lerp(ks, fks, rs);
}

// Evaluates the GGX distribution and geometric term
inline float eval_ggx(float rs, float ndh, float ndi, float ndo) {
  // evaluate GGX
  auto alpha2 = rs * rs;
  auto di = (ndh * ndh) * (alpha2 - 1) + 1;
  auto d = alpha2 / (pif * di * di);

  auto go = (2 * ndo) / (ndo + sqrt(alpha2 + (1 - alpha2) * ndo * ndo));
  auto gi = (2 * ndi) / (ndi + sqrt(alpha2 + (1 - alpha2) * ndi * ndi));
  auto g = go * gi;

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
vec3f eval_triangle_brdfcos(const point& pt, const vec3f& i) {
  auto brdfcos = zero3f;

  const auto& o = pt.o;
  const auto& n = pt.n;
  auto h = normalize(o + i);

  // compute dot products
  auto ndo = dot(n, o), ndi = dot(n, i),
      ndh = dot(h, n);

  brdfcos = (pt.kd/pif)+
      ((pt.ks* eval_ggx(pt.rs, ndh, ndi, ndo))/(4*ndi*ndo));

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

/// Evaluate the BSDF*cosine as discussed in the slides
vec3f sample_triangle_brdfcos(const point& pt, rng_t& rng) {
  //TODO sample_triangle_brdfcos

  auto& wn = pt.n;
  auto& wo = pt.o;
  auto& fp = pt.ist->frame;

  // skip if no component
  if (!pt.hit()) return zero3f;

  // probability of each lobe
  auto kdw = max_element_val(pt.kd), ksw = max_element_val(pt.ks);
  auto kaw = kdw + ksw;
  kdw /= kaw;
  ksw /= kaw;

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
    return transform_direction(fp, wi_local);
  }
    // sample according to specular GGX
  else if (rnl < kdw + ksw) {
    // sample wh with ggx distribution
    auto wh_local = sample_ggx(pt.rs, rn);
    auto wh = transform_direction(fp, wh_local);
    // compute wi
    return normalize(wh * 2.0f * dot(wo, wh) - wo);
  }
    // transmission hack
  else if (rnl < kdw + ksw)
    // continue ray direction
    return -wo;

  return {255,255,255};
}

/// Comute the weight for BSDF sampling, i.e. 1 / pdf.
float weight_triangle_brdfcos(const point& pt, const vec3f& i) {
  //TODO weight_triangle_brdfcos

  return {};
}

vec3f sample_spherical_dir(const point& pt, rng_t& rng) {
  auto rn = vec2f{next_rand1f(rng), next_rand1f(rng)};
  auto rz = rn.y, rr = sqrtf(1 - rz * rz), rphi = 2 * pif * rn.x;
  auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
  return transform_direction(make_frame3_fromz(pt.x, pt.n), wi_local);
}

float weight_spherical_dir() { return 1 / (4 * pif); }

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
//          auto li = estimate_li(scn, ray.o, ray.d, bounces, rng);


  auto pt = intersect(scn, q, d);
  if(!pt.hit()) return pt.le;
  if(bounces <= 0) return pt.le;
  auto i = sample_brdfcos(pt, rng);
  auto li = estimate_li_naive(scn, pt.x,i,bounces-1,rng);
  auto lr = li * eval_brdfcos(pt,i) / sample_hemisphere_cosine_pdf(i);
  return pt.le + lr;

  /***
   * vec3f estimate_li(scene* scn, vec3f q, vec3f i) {
     auto pt = intersect(scn, {q, i});
    if(!pt.f) return pt.le;
    if(bounce >= max_bounce) return pt.le;
    auto i = sample_brdfcos(pt);
    auto li = estimate_li(scn, {pt.x,i});
    auto lr = li * eval_brdfcos(pt,i) / pdf(i);
    return le + lr;
}
   */
}

/// Produce formulation of pathtracing that matches exactly eh above.
/// In this method, use hemispherical cosine sampling and only lambert BSDF.
vec3f estimate_li_product(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
  //TODO estimate_li_product
  return {};
}

/// Pathtracing with direct+indirect and russian roulette
vec3f estimate_li_direct(
    const scene* scn, const vec3f& q, const vec3f& d, int bounces, rng_t& rng) {
  //TODO estimate_li_direct
  return {};
}

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
