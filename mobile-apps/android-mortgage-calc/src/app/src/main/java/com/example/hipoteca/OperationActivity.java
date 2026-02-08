package com.example.hipoteca;

import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import com.example.hipoteca.R;

/**
 *  Clase secundaria que calcula los costes hipotecarios recibiendo los parámetros introducidos por el usuario.
 *
 *  @author Manuel Lorente Almán
 *  @since 08-04-2020
 *  @version 1.0
 */
public class OperationActivity extends AppCompatActivity {
	/**
     * Definicion de los manejadores para los distintos parametros:
     * PRECIO - Variable par almacenar información del contexto: precio del inmueble
     * AHORRO - Variable para almacenar información del contexto: ahorro aportado
     * INTERES - Variable para almacenar información del contexto: interés de la hipoteca
     * PLAZO - Variable para almacenar información del contexto: plazo para la hipoteca
     */
    private static final String PRECIO = "PRECIO";
    private static final String AHORRO = "AHORRO";
    private static final String INTERES = "INTERES";
    private static final String PLAZO = "PLAZO";
	
    /**
     * Constructor de la clase 
     * @param savedInstanceState Bundle del contexto
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_operation);
		// Se recuperan los datos de la vista anterior
        Bundle data = this.getIntent().getExtras();

        // El precio es un entero
        int precio = Integer.parseInt(data.getString("PRECIO").substring(0,data.getString("PRECIO").length()-2));
        // El ahorro es un porcentaje
        double ahorro = Integer.parseInt(data.getString("AHORRO").substring(0,data.getString("AHORRO").length()-2));
        // El interes es un porcentaje
        double interes = Integer.parseInt(data.getString("INTERES").substring(0,data.getString("INTERES").length()-2));
        // El plazo es un entero
        int plazo = Integer.parseInt(data.getString("PLAZO").substring(0,data.getString("PLAZO").length()-5));
        // Impuestos (13%)
        int impuestos = 13;

        // Mensualidad = Formula pdf
        double ahorro_aportado = precio*ahorro/100;
        double i = interes*0.01/12;
        double n = plazo*12;
        double p = precio - ahorro_aportado;
        double mensualidad =  p/((1 - Math.pow(1+i,-n))/i);
		
        // Importe total de la hipoteca
        double totalHipoteca = mensualidad*(plazo*12);
		
        // Interés hipoteca = Importe total de la hipoteca - precio - ahorro)
        double totalInteres = totalHipoteca - p;
        double gastos = precio*impuestos/100;
		
        // Coste total con hipoteca = total hipoteca + gastos e impuestos + ahorro
        double totalConHipoteca = totalHipoteca + gastos + ahorro_aportado;
		
        // Coste total de la compra = precio inmueble + impuestos y gastos(13%)
        double totalCompra = precio + gastos;

		// Actualizamos la vista con los valores calculados
        TextView totalHipotecaID = findViewById(R.id.totalValue);
        totalHipotecaID.setText(String.format ("%.2f", totalHipoteca) +" €");
        TextView mensualidadID = findViewById(R.id.mensualValue);
        mensualidadID.setText(String.format ("%.2f", mensualidad) +" €");

        TextView totalCompraID = findViewById(R.id.totalCompraValue);
        totalCompraID.setText(String.format ("%.2f", totalCompra) +" €");
        TextView precioInmuebleID = findViewById(R.id.totalPrecioValue);
        precioInmuebleID.setText(String.format ("%d", precio) +" €");
        TextView impuestosID = findViewById(R.id.impuestosValue);
        impuestosID.setText(String.format ("%.2f", gastos) +" €");

        TextView totalConHipotecaID = findViewById(R.id.totalConHipotecaValue);
        totalConHipotecaID.setText(String.format ("%.2f", totalConHipoteca) +" €");
        TextView ahorroID = findViewById(R.id.ahorroValue);
        ahorroID.setText(String.format ("%.2f", ahorro_aportado) +" €");
        TextView totalHipotecaID2 = findViewById(R.id.totalHipotecaValue);
        totalHipotecaID2.setText(String.format ("%.2f", totalHipoteca) +" €");
        TextView totalInteresID = findViewById(R.id.totalInteresValue);
        totalInteresID.setText(String.format ("%.2f", totalInteres) +" €");

    }
}
