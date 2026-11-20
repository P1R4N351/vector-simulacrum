// Run only in an isolated replay process. Delegate collision detection to MuJoCo.
#include <mujoco/mujoco.h>
#include <string.h>

static int enabled;
void enable_tread_frame(int value) {enabled=value;}

static mjfCollision original[mjNGEOMTYPES][mjNGEOMTYPES];

static int tread_frame(const mjModel* m, mjData* d, mjPreContact* con,
                       int a, int b, mjtNum margin) {
  int n = original[m->geom_type[a]][m->geom_type[b]](m,d,con,a,b,margin);
  if (!enabled || !n) return n;
  const char* an = mj_id2name(m,mjOBJ_GEOM,a);
  const char* bn = mj_id2name(m,mjOBJ_GEOM,b);
  if (!an || !bn) return n;
  if (!((!strcmp(an,"table") && !strncmp(bn,"wheel_patch_",12)) ||
        (!strcmp(bn,"table") && !strncmp(an,"wheel_patch_",12)))) return n;
  int patch = !strcmp(an,"table") ? b : a;
  int chassis = m->body_parentid[m->geom_bodyid[patch]];
  const mjtNum* R = d->xmat+9*chassis;
  mjtNum forward[3] = {R[0],R[3],R[6]};
  for (int i=0;i<n;i++) {
    mjtNum dot = mju_dot3(forward,con[i].normal);
    mju_addScl3(con[i].tangent,forward,con[i].normal,-dot);
    mju_normalize3(con[i].tangent);
  }
  return n;
}

void install_tread_frame(void) {
  int types[2][2] = {{mjGEOM_SPHERE,mjGEOM_BOX},{mjGEOM_CYLINDER,mjGEOM_BOX}};
  for (int i=0;i<2;i++) {
    int a=types[i][0], b=types[i][1];
    if (a>b) {int tmp=a; a=b; b=tmp;}
    if (!original[a][b]) {
      original[a][b]=mjCOLLISIONFUNC[a][b];
      mjCOLLISIONFUNC[a][b]=tread_frame;
    }
  }
}
