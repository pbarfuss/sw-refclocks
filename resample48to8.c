/* 8 to 48 kHz sample rate conversion defines */
#define C2_OS                 6         /* oversampling rate           */
#define C2_OS_TAPS           48         /* number of OS filter taps    */

/* Generate using fir1(47,1/6) in Octave */
const float fdmdv_os_filter[]= {
    -3.55606818e-04,
    -8.98615286e-04,
    -1.40119781e-03,
    -1.71713852e-03,
    -1.56471179e-03,
    -6.28128960e-04,
     1.24522223e-03,
     3.83138676e-03,
     6.41309478e-03,
     7.85893186e-03,
     6.93514929e-03,
     2.79361991e-03,
    -4.51051400e-03,
    -1.36671853e-02,
    -2.21034939e-02,
    -2.64084653e-02,
    -2.31425052e-02,
    -9.84218694e-03,
     1.40648474e-02,
     4.67316298e-02,
     8.39615986e-02,
     1.19925275e-01,
     1.48381174e-01,
     1.64097819e-01
};

const float fdmdv_os_filter2[]= {
    -7.8177e-05, -6.5108e-04, -1.2444e-03, -1.7422e-03, -1.8821e-03, -1.3180e-03,  2.1001e-04,  2.6551e-03,  5.4962e-03,
     7.7305e-03,  8.0917e-03,  5.4769e-03, -5.2844e-04, -9.2047e-03, -1.8523e-02, -2.5374e-02, -2.6211e-02, -1.7990e-02,
     8.4704e-04,  2.9640e-02,  6.5285e-02,  1.0264e-01,  1.3553e-01,  1.5809e-01,  1.6611e-01
};

const float fdmdv_os_filter_small[] = {
    -1.2226e-03,
    -2.7618e-03,
    -5.4023e-03,
    -8.0971e-03,
    -7.6258e-03,
     4.6280e-04,
     1.9757e-02,
     5.0674e-02,
     8.9144e-02,
     1.2716e-01,
     1.5517e-01,
     1.6549e-01
};

void codec2_48_to_8(float *out8k, float *in48k, unsigned int n)
{
    unsigned int i, j;

    for(i=0; i<n; i++) {
      out8k[i] = 0.0;
      for(j=0; j<24; j++) {
        out8k[i] += fdmdv_os_filter[j]*(in48k[6*i - j] + in48k[6*i + 47 - j]);
      }
    }

    /* update filter memory */
    for(i=0; i<48; i++)
        in48k[i + 48] = in48k[i + 48*n + 48];
}

