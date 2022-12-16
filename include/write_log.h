#pragma once

inline void time_log(FILE *fp, double toc) {
    fprintf(fp, "%lf", toc);
    fprintf(fp, "\n");

    fflush(fp);
}

inline void size_log(FILE *fp, size_t size) {
    fprintf(fp, "%zd", size);
    fprintf(fp, "\n");

    fflush(fp);
}