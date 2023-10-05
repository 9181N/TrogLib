struct path {
    double x[1000];
    double y[1000];
    double h[1000];
    double d2x[1000];
    double d2y[1000];
    double d2slope[1000];
    double length{0};
    double fidelity{100};
};

extern struct path path1;

void generate_cubic_values(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float fidelity);
void print_cubic(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, int fidelity);