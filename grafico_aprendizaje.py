import pandas as pd
import matplotlib.pyplot as plt

REPORTE_CSV_FILE = "reporte_aprendizaje.csv"

def generar_grafico_de_aprendizaje():
    try:
        # 1. Cargar los datos del reporte usando Pandas
        df = pd.read_csv(REPORTE_CSV_FILE)
        
        # Asegurarse de que la columna de recompensa sea numérica
        df['Recompensa'] = pd.to_numeric(df['Recompensa'])

        # 2. Calcular la "Recompensa Acumulada Móvil"
        # Esto suaviza la curva y muestra la tendencia de aprendizaje a lo largo del tiempo.
        # Una ventana de 10 significa que cada punto es el promedio de las últimas 10 recompensas.
        df['Recompensa Acumulada'] = df['Recompensa'].rolling(window=10).mean()

        # 3. Generar el gráfico con Matplotlib
        plt.figure(figsize=(12, 7))
        plt.plot(df['Paso de Tiempo'], df['Recompensa Acumulada'], marker='o', linestyle='-', color='b')
        
        plt.title('Curva de Aprendizaje del Cerebro (Recompensa Promedio a lo Largo del Tiempo)', fontsize=25)
        plt.xlabel('Pasos de Aprendizaje', fontsize=25)
        plt.ylabel('Recompensa Promedio (Ventana de 10 pasos)', fontsize=2512)
        plt.grid(True)
        plt.show()
        
        print("Gráfico de aprendizaje generado con éxito.")

    except FileNotFoundError:
        print(f"Error: No se encontró el archivo '{REPORTE_CSV_FILE}'.")
    except Exception as e:
        print(f"Ocurrió un error al generar el gráfico: {e}")

if __name__ == "__main__":
    generar_grafico_de_aprendizaje()