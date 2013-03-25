/*********************************************************************
 *  File:         transform_math.c
 *  Description:  some procedures for doing homogeneous transform math
 *  Author:       Rod Grupen
 *  Date:         2-28-89
 *********************************************************************/

T_mult(t1,t2,result)
double t1[4][4],t2[4][4],result[4][4];
{
  int i,j,k;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      result[i][j] = 0.0;
      for (k=0; k<4; ++k) {
	result[i][j] += t1[i][k] * t2[k][j];
      }
    }
  }
}

/* produce the output inverted homogeneous transform for the input transform */
inv_transform(in, out)
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<3; ++i) {
    for (j=0; j<3; ++j) {
      out[i][j] = in[j][i];
    }
  }
  out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;

  out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
  out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
  out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}

T_copy(t1,t2)                  /* copy t1 into t2 */
double t1[4][4],t2[4][4];
{
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      t2[i][j] = t1[i][j];
    }
  }
}

apply_transform(in, trans, out)
double in[4], trans[4][4], out[4];
{
  int i,j;
  double sum;
  for (i=0; i<4; ++i) {
    sum = 0.0;
    for (j=0; j<4; ++j) {
      sum += trans[i][j] * in[j];
    }
    out[i] = sum;
  }
}
