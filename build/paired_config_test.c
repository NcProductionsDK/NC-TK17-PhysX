
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
typedef struct {
 float max_angle,link_max_angle[3][3],link_min_angle[3][3],link_gain[3];
 unsigned collision_offset_bounds;
 float collision_min_offset[3],collision_max_offset[3];
} body_chain_physics_config_t;
static void log_line(const char *fmt,...){(void)fmt;}
static void trim_in_place(char *s)
{
    char *e;
    if (!s) return;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') memmove(s, s + 1, strlen(s));
    e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) *--e = 0;
}
static int profile_string_found(const char *section,
                                const char *key,
                                char *out,
                                DWORD out_size,
                                const char *path)
{
    static const char sentinel[] = "\x1f__missing__\x1f";
    if (!section || !key || !out || out_size == 0) return 0;
    GetPrivateProfileStringA(section, key, sentinel, out, out_size, path);
    return strcmp(out, sentinel) != 0;
}
static float profile_float(const char *section, const char *key, float fallback, const char *path)
{
    char buf[128];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    if (!buf[0]) return fallback;
    return (float)atof(buf);
}
static void fill_vec3(float out[3], float value)
{
    if (!out) return;
    out[0] = value;
    out[1] = value;
    out[2] = value;
}
static void profile_vec3_or_float(const char *section,
                                  const char *key,
                                  float fallback,
                                  float out[3],
                                  const char *path)
{
    char buf[128];
    float x, y, z;
    if (!out) return;
    fill_vec3(out, fallback);
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return;
    if (sscanf(buf, "%f,%f,%f", &x, &y, &z) == 3 ||
        sscanf(buf, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
        return;
    }
    x = (float)atof(buf);
    fill_vec3(out, x);
}
static void profile_min_angle_vec3_or_float(const char *section,
                                            const char *key,
                                            const float max_angle[3],
                                            float out[3],
                                            const char *path)
{
    char buf[128];
    float x, y, z;
    int axis;
    if (!out || !max_angle) return;
    for (axis = 0; axis < 3; axis++) {
        out[axis] = -max_angle[axis];
    }
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return;
    if (sscanf(buf, "%f,%f,%f", &x, &y, &z) == 3 ||
        sscanf(buf, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
        return;
    }
    x = (float)atof(buf);
    fill_vec3(out, x);
}
static void profile_paired_angle_settings(const char *section, const char *path,
    body_chain_physics_config_t *cfg, int overlay)
{
    char value[128];
    const char *max_key = profile_string_found(section, "max_angle", value, sizeof(value), path)
        ? "max_angle" : "joint01_max_angle";
    const char *min_key = profile_string_found(section, "min_angle", value, sizeof(value), path)
        ? "min_angle" : "joint01_min_angle";
    const char *gain_key = profile_string_found(section, "gain", value, sizeof(value), path)
        ? "gain" : "joint01_gain";
    int max_present = profile_string_found(section, max_key, value, sizeof(value), path);
    int min_present = profile_string_found(section, min_key, value, sizeof(value), path);
    if (!overlay || max_present)
        profile_vec3_or_float(section, max_key, cfg->max_angle, cfg->link_max_angle[0], path);
    if (!overlay || max_present || min_present)
        profile_min_angle_vec3_or_float(section, min_key, cfg->link_max_angle[0],
                                        cfg->link_min_angle[0], path);
    cfg->link_gain[0] = profile_float(section, gain_key, overlay ? cfg->link_gain[0] : 1.0f, path);
}
static void profile_paired_collision_offsets(const char *section, const char *path,
    body_chain_physics_config_t *cfg, int overlay)
{
    const char *keys[2] = {"collision_min_offset", "collision_max_offset"};
    int bound, axis;
    if (!overlay) {
        cfg->collision_offset_bounds = 0;
        memset(cfg->collision_min_offset, 0, sizeof(cfg->collision_min_offset));
        memset(cfg->collision_max_offset, 0, sizeof(cfg->collision_max_offset));
    }
    for (bound = 0; bound < 2; bound++) {
        char text[128], *cursor, *end;
        float values[3];
        if (!profile_string_found(section, keys[bound], text, sizeof(text), path)) continue;
        cursor = text;
        for (axis = 0; axis < 3; axis++) {
            values[axis] = strtof(cursor, &end);
            if (end == cursor || !isfinite(values[axis]) ||
                (bound ? values[axis] < 0 : values[axis] > 0)) break;
            cursor = end;
            while (*cursor == ' ' || *cursor == '\t') cursor++;
            if (axis < 2) {
                if (*cursor == ',') cursor++;
                else if (cursor == end) break;
            }
        }
        /* A whole XYZ tuple is required. Zero must be inside the range,
           otherwise a bound would move a bone even without any collision. */
        if (axis != 3 || (*cursor && *cursor != ';')) {
            log_line("settings ignored [%s] %s reason=\"expected three finite XYZ offsets; minimum <= 0, maximum >= 0\"",
                     section, keys[bound]);
            continue;
        }
        memcpy(bound ? cfg->collision_max_offset : cfg->collision_min_offset,
               values, sizeof(values));
        cfg->collision_offset_bounds |= 1u << bound;
    }
}

static void assert_close(float a,float b){assert(fabsf(a-b)<1e-6f);}
int main(int argc,char **argv){
 assert(argc==3);
 const char *sections[]={"breasts_physics","butt_physics"};
 for(int s=0;s<2;s++){
  const char *section=sections[s];
  assert(WritePrivateProfileStringA(section,NULL,NULL,argv[1]));
  assert(WritePrivateProfileStringA(section,NULL,NULL,argv[2]));
  body_chain_physics_config_t cfg={0};cfg.max_angle=30;
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_max_angle[0][0],30);assert_close(cfg.link_min_angle[0][0],-30);assert_close(cfg.link_gain[0],1);
  WritePrivateProfileStringA(section,"joint01_max_angle","10,20,30",argv[1]);
  WritePrivateProfileStringA(section,"joint01_min_angle","-5,-15,-25",argv[1]);
  WritePrivateProfileStringA(section,"joint01_gain","0.7",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  for(int a=0;a<3;a++){assert_close(cfg.link_max_angle[0][a],10+10*a);assert_close(cfg.link_min_angle[0][a],-5-10*a);}
  assert_close(cfg.link_gain[0],.7f);
  WritePrivateProfileStringA(section,"max_angle","40,50,60",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_max_angle[0][2],60);assert_close(cfg.link_min_angle[0][2],-25);assert_close(cfg.link_gain[0],.7f);
  WritePrivateProfileStringA(section,"min_angle","-12",argv[1]);
  WritePrivateProfileStringA(section,"gain","1.2",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_min_angle[0][2],-12);assert_close(cfg.link_gain[0],1.2f);
  /* Legacy sidecar overrides the global config, even when global uses aliases. */
  WritePrivateProfileStringA(section,"joint01_gain","0.5",argv[2]);
  profile_paired_angle_settings(section,argv[2],&cfg,1);
  assert_close(cfg.link_gain[0],.5f);assert_close(cfg.link_max_angle[0][2],60);assert_close(cfg.link_min_angle[0][2],-12);
  WritePrivateProfileStringA(section,"max_angle","15",argv[2]);
  WritePrivateProfileStringA(section,"gain","0.9",argv[2]);
  profile_paired_angle_settings(section,argv[2],&cfg,1);
  assert_close(cfg.link_gain[0],.9f);assert_close(cfg.link_max_angle[0][2],15);assert_close(cfg.link_min_angle[0][2],-15);
  WritePrivateProfileStringA(section,NULL,NULL,argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_gain[0],1);assert_close(cfg.link_max_angle[0][2],30);assert_close(cfg.link_min_angle[0][2],-30);

  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
  WritePrivateProfileStringA(section,"collision_min_offset","-0.01, -0.02, 0 ; comment",argv[1]);
  WritePrivateProfileStringA(section,"collision_max_offset","0.03 0.04 0.05",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==3);
  assert_close(cfg.collision_min_offset[0],-.01f);assert_close(cfg.collision_min_offset[1],-.02f);assert_close(cfg.collision_min_offset[2],0);
  assert_close(cfg.collision_max_offset[0],.03f);assert_close(cfg.collision_max_offset[2],.05f);
  WritePrivateProfileStringA(section,"collision_max_offset","0,0,0",argv[2]);
  profile_paired_collision_offsets(section,argv[2],&cfg,1);
  assert_close(cfg.collision_min_offset[0],-.01f);assert_close(cfg.collision_max_offset[2],0);
  const char *bad[]={"nan,0,0","0,inf,0","0.01","0,0","0,0,0junk","0,0,0,0","0-1-2","1,0,0"};
  for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){
   WritePrivateProfileStringA(section,"collision_min_offset",bad[i],argv[2]);
   profile_paired_collision_offsets(section,argv[2],&cfg,1);
   assert_close(cfg.collision_min_offset[0],-.01f);assert(cfg.collision_offset_bounds==3);
  }
  WritePrivateProfileStringA(section,"collision_min_offset",NULL,argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==2);
  WritePrivateProfileStringA(section,"collision_max_offset","-0.1,0,0",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
  WritePrivateProfileStringA(section,"collision_min_offset","0,0,0",argv[1]);
  WritePrivateProfileStringA(section,"collision_max_offset","0,0,0",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==3);
  WritePrivateProfileStringA(section,NULL,NULL,argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
 }
 puts("PASS: breast/butt aliases take precedence per key, legacy keys work globally and in sidecars, scalar/vector angles and missing-key defaults remain supported");
 puts("PASS: optional XYZ collision bounds inherit independently, zero is valid, malformed/non-finite/wrong-sign input is rejected, removal restores legacy behavior");
 return 0;
}
