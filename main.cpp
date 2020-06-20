#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"

struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &cameraPoint, const Vec3f &dir, float &t1) const {  //Проверка попадания луча в сферу
        Vec3f O = center - cameraPoint;    // Луч, идущий из камеры к центру сферы
        float l1 = O*dir; // Получаем длину отрезка от камеры до точки перечечения с перпендикулярной плоскостью, проходящей через центр сферы 
        float d2 = O*O - l1*l1; // Находим квадрат перпендикуляра от центра сферы до выпущенного луча
        if (d2 > radius*radius) return false; // Если длина перпендикуляра больше радиуса сферы, значит луч не попадает

        float l2 = sqrtf(radius*radius - d2); // Длина отрезка луча от поверхности сферы до точки пересечения перпендикуляром
        t1 = l1 - l2; // Расстояние от камеры до поверхности сферы
        float t2 = l1 + l2;

        if (t1 < 0) t1 = t2;    // Проверяем, что камера перед первой точкой пересечения со сферой
        if (t1 < 0) return false;   // Если камера позади первой и второй точек пересечения, значит сфера позади камеры, пересечение не учитывается

        return true;
    }
};

Vec3f reflect(const Vec3f &L, const Vec3f &N) {           // Функция для вычисления направления отраженного луча по модели Фонга
    return L - N*2.f*(L*N);                               // N - вектор нормали в точке, L - направление из точки на источник
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Расчет преломления по закону Снеллиуса
    float cosi = - std::max(-1.f, std::min(1.f, I*N)); // Находим угол падения света 
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // если луч исходит изнутри объекта, меняем местами среду объекта и среду окружения
    float eta = eta_i / eta_t; // eta_i и eta_t - коэффициенты преломления внешней среды и среды объекта соответственно
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return  I*eta + N*(eta*cosi - sqrtf(k)); 
}

bool scene_intersect(const Vec3f &cameraPoint, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) { // Поиск пересечения с ближайшей сферой
    float spheres_dist = std::numeric_limits<float>::max();
    for (int i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(cameraPoint, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = cameraPoint + dir*dist_i; // Находим точку пересечения сферы лучем
            N = (hit - spheres[i].center).normalize(); // Рассчитываем вектор нормали к точке пересечения
            material = spheres[i].material;
        }
    }

    return spheres_dist<1000; 
}

Vec3f ray_cast(const Vec3f &cameraPoint, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, int depth = 0) { // Отправка луча, возвращает цвет пикселя
    Vec3f point, N;
    Material material;

    if (depth>4 || !scene_intersect(cameraPoint, dir, spheres, point, N, material)) {
        return Vec3f(0.3, 0.3, 0.5);    // Если луч никуда не попадает или превышена глубина рекурсии (необходима для избежания бесконечного цикла), пикселю задается цвет фона
    }

    Vec3f reflection_dir = reflect(dir, N).normalize();    // Расчет траектории отражения и преломления луча
    Vec3f refraction_dir = refract(dir, N, material.refractive_index).normalize();

    Vec3f reflection_point = reflection_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;   // Смещение исходной точки, чтобы объект не отражался сам на себя
    Vec3f refraction_point = refraction_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;

    Vec3f reflect_color = ray_cast(reflection_point, reflection_dir, spheres, lights, depth + 1); // Отправка отраженного/преломленного луча с увеличением глубины рекурсии на 1
    Vec3f refract_color = ray_cast(refraction_point, refraction_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;

    for (int i=0; i<lights.size(); i++) {    // В месте попадания луча просчитывается освещение от всех источников
        Vec3f light_dir = (lights[i].position - point).normalize();

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N); // Расчет рассеяного света
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity; // Расчет бликов
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1.f, 1.f, 1.f)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights, const int width, const int height) {

    const float fov = M_PI/3.;
    std::vector<Vec3f> screenBuffer(width*height); // Буфер, в котором будет сохраняться цвет для каждой ячейки (пикселя) для последующего преобразования в изображение

    #pragma omp parallel for // Параллельный цикл для ускорения вычислений
    for (size_t j = 0; j<height; j++) {  // Циклично отправляются лучи из каждой ячейки
        for (size_t i = 0; i<width; i++) {
            float dir_x =  (i + 0.5) -  width/2.;
            float dir_y = -(j + 0.5) + height/2.;    
            float dir_z = -height/(2.*tan(fov/2.));
            screenBuffer[i+j*width] = ray_cast(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }

    std::ofstream ofs; // Запись значений из буфера в pmm изображение
    ofs.open("./output.ppm",std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = screenBuffer[i];

        float max = std::max(c[0], std::max(c[1], c[2])); // Ищем максимальное значение в RGB векторе

        if (max>1) // Если максимальное значение больше единицы, делим на него единицу для корректной записи в файл
            c = c*(1./max);

        for (int j = 0; j<3; j++) {
            ofs << (char)(255 * screenBuffer[i][j]);
        }
    }
    ofs.close();
}

int main() {
    Material marble(1.0, Vec4f(0.5, 0.2, 0.2, 0.0), Vec3f(0.93, 0.93, 0.91), 500.);
    Material wood(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.52, 0.37, 0.26), 10.);
    Material mirror(1.0, Vec4f(0.0, 150.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1500.);
    Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.5, 0.5, 0.5), 100.);
    

    std::vector<Sphere> spheres;

    spheres.push_back(Sphere(Vec3f(3, 0, -25), 5, wood));
    spheres.push_back(Sphere(Vec3f(1, 0, -15), 3, mirror));
    spheres.push_back(Sphere(Vec3f(1, 0, 15), 3, marble));


    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(0, 10,  10), 1));
    lights.push_back(Light(Vec3f( 20, -10, -25), 1.5));


    render(spheres, lights, 1024, 768);

    return 0;
}