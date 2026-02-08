package com.example.hipoteca;

import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;

import androidx.appcompat.app.AppCompatActivity;

import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

/**
 *  Clase principal que permite calcular los costes hipotecarios de la compra de una vivienda
 *  nueva.
 *
 *  @author Manuel Lorente Almán
 *  @since 08-04-2020
 *  @version 1.0
 */
public class MainActivity extends AppCompatActivity {
    /**
     * Definicion de los manejadores para los distintos parametros:
     * precioTV - Manejador en la vista principal para el visor del precio del inmueble
     * precioSB - Manejador en la vista principal para la barra del precio del inmueble
     * ahorroTV - Manejador en la vista principal para el visor del ahorro aportado
     * ahorroSB - Manejador en la vista principal para la barra del ahorro aportado
     * interesTV - Manejador en la vista principal para el visor del interés de la hipoteca
     * interesSB - Manejador en la vista principal para la barra del interés de la hipoteca
     * plazoTV - Manejador en la vista principal para el visor del plazo para la hipoteca
     * plazoSB - Manejador en la vista principal para la barra del plazo para la hipoteca
     * minimoAhorroTV - Manejador en la vista principal para el visor del mensaje de aviso
     * botonCalc - Manejador en la vista principal para el botón de cálculo de costes
     * PRECIO - Variable par almacenar información del contexto: precio del inmueble
     * AHORRO - Variable para almacenar información del contexto: ahorro aportado
     * INTERES - Variable para almacenar información del contexto: interés de la hipoteca
     * PLAZO - Variable para almacenar información del contexto: plazo para la hipoteca
     */
    TextView precioTV;
    SeekBar precioSB;
    TextView ahorroTV;
    SeekBar ahorroSB;
    TextView interesTV;
    SeekBar interesSB;
    TextView plazoTV;
    SeekBar plazoSB;
    TextView minimoAhorroTV;
    Button  botonCalc;

    private static final String PRECIO = "PRECIO";
    private static final String AHORRO = "AHORRO";
    private static final String INTERES = "INTERES";
    private static final String PLAZO = "PLAZO";

    /**
     * Método para guardar el contexto de los datos introducidos.
     * @param savedInstanceState Bundle que se pasará al método onCreate si el proceso se mata y
     *                           reinicia
     */
    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        savedInstanceState.putString("PRECIO", precioTV.getText().toString());
        savedInstanceState.putString("AHORRO", ahorroTV.getText().toString());
        savedInstanceState.putString("INTERES", interesTV.getText().toString());
        savedInstanceState.putString("PLAZO", plazoTV.getText().toString());
    }

    /**
     * Método para restaurar el contexto de los datos introducidos.
     * @param savedInstanceState Bundle con los datos para restaurar el contexto.
     */
    @Override
    public void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        precioTV.setText(savedInstanceState.getString("PRECIO"));
        ahorroTV.setText(savedInstanceState.getString("AHORRO"));
        interesTV.setText(savedInstanceState.getString("INTERES"));
        plazoTV.setText(savedInstanceState.getString("PLAZO"));
    }

    /**
     * Constructor de la clase principal del programa
     * @param savedInstanceState Bundle del contexto
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Precio del inmueble - Min = 0 Max = 500000 Step = 500
        precioTV = findViewById(R.id.precioView);
        precioSB = findViewById(R.id.precioSeekBar);
        precioSB.incrementProgressBy(500);
        precioSB.setMax(500000);

        // Ahorro aportado - Min = 30 Max = 100
        ahorroTV = findViewById(R.id.ahorroView);
        ahorroSB = findViewById(R.id.ahorroSeekBar);
        ahorroSB.setMax(100);
        minimoAhorroTV = findViewById(R.id.minimoAhorroText);

        // Tipo de interés - Min = 0 Max = 10
        interesTV = findViewById(R.id.interesView);
        interesSB = findViewById(R.id.interesSeekBar);
        interesSB.setMax(10);

        // Plazo de años - Min = 0 Max = 40
        plazoTV = findViewById(R.id.plazoView);
        plazoSB = findViewById(R.id.plazoSeekBar);
        plazoSB.setMax(40);

        // Handler del boton - Deshabilitado si el ahorro aportado < 30%
        botonCalc = findViewById(R.id.calcularButton);
        botonCalc.setEnabled(false);
        botonCalc.setClickable(false);
        botonCalc.getBackground().setAlpha(64);
        botonCalc.setTextColor(Color.argb(64,0,0,0));

        /**
         *  Se ejecuta cada vez que se interactúa con la barra del precio del inmueble
         */
        precioSB.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            /**
             * Método que se ejecuta cuando la barra está en movimiento
             * @param seekBar Manejador de la barra
             * @param value Valor de la barra actual
             * @param b True si el cambio lo inció el usuario
             */
            @Override
            public void onProgressChanged(SeekBar seekBar, int value, boolean b) {
				// Aquí se modifica el valor para que el paso sea de 500€
				// https://stackoverflow.com/questions/7329166/changing-step-values-in-seekbar
                value = value / 500;
                value = value * 500;
                updateSB(1, value);
            }
            /**
             * Método que se ejecuta cuando la barra inicia el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            /**
             * Método que se ejecuta cuando la barra detiene el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });

        /**
         *  Se ejecuta cada vez que se interactúa con la barra del ahorro aportado
         */
        ahorroSB.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			/**
             * Método que se ejecuta cuando la barra está en movimiento
             * @param seekBar Manejador de la barra
             * @param value Valor de la barra actual
             * @param b True si el cambio lo inció el usuario
             */
            @Override
            public void onProgressChanged(SeekBar seekBar, int value, boolean b) {
                updateSB(2, value);
            }
            /**
             * Método que se ejecuta cuando la barra inicia el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            /**
             * Método que se ejecuta cuando la barra detiene el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });
		
        /**
         *  Se ejecuta cada vez que se interactúa con la barra del interés de la hipoteca
         */
        interesSB.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			/**
             * Método que se ejecuta cuando la barra está en movimiento
             * @param seekBar Manejador de la barra
             * @param value Valor de la barra actual
             * @param b True si el cambio lo inció el usuario
             */
            @Override
            public void onProgressChanged(SeekBar seekBar, int value, boolean b) {
                updateSB(3, value);
            }
            /**
             * Método que se ejecuta cuando la barra inicia el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            /**
             * Método que se ejecuta cuando la barra detiene el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });

        /**
         *  Se ejecuta cada vez que se interactúa con la barra del plazo para la hipoteca
         */
        plazoSB.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			/**
             * Método que se ejecuta cuando la barra está en movimiento
             * @param seekBar Manejador de la barra
             * @param value Valor de la barra actual
             * @param b True si el cambio lo inció el usuario
             */	
            @Override
            public void onProgressChanged(SeekBar seekBar, int value, boolean b) {
                updateSB(4, value);
            }
            /**
             * Método que se ejecuta cuando la barra inicia el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }
            /**
             * Método que se ejecuta cuando la barra detiene el movimiento
             * @param seekBar Manejador de la barra
             */
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });
    }
	/**
	 * Método para actualizar el valor de la barra de progreso en cada caso
	 * @param seekbar Identificador del parámetro a actualizar
	 * @param value Valor a mostrar en la vista
	 */	
    public void updateSB(int seekbar, int value){
        switch (seekbar) {
			// Precio del inmbueble
            case 1:
                precioTV.setText(value + " €");
                break;
			// Ahorro aportado
            case 2:
                ahorroTV.setText(value + " %");
				// Se comprueba si está entre 30 y 99. En caso contrario muestra mensaje de erro
				// y deshabilita el botón de cálculo de costes hipotecarios
                if (value < 30) {
                    botonCalc.setEnabled(false);
                    botonCalc.setClickable(false);
                    botonCalc.getBackground().setAlpha(64);
                    botonCalc.setTextColor(Color.argb(64,0,0,0));
                    minimoAhorroTV.setText("Se requiere un ahorro mínimo del 30%");
                }
                else if (value == 100) {
                    botonCalc.setEnabled(true);
                    botonCalc.setClickable(true);
                    botonCalc.getBackground().setAlpha(255);
                    botonCalc.setTextColor(Color.argb(255,0,0,0));
                    minimoAhorroTV.setText("El ahorro aportado debe ser menor que el importe del inmueble");
                }
                else {
                    botonCalc.setEnabled(true);
                    botonCalc.setClickable(true);
                    botonCalc.getBackground().setAlpha(255);
                    botonCalc.setTextColor(Color.argb(255,0,0,0));
                    minimoAhorroTV.setText("");
                }
                break;
			// Interés de la hipoteca
            case 3:
                interesTV.setText(value + " %");
                break;
			// Plazo en años de la hipoteca
            default:
                plazoTV.setText(value + " años");
                break;
        }
    }
	/**
	 * Método para iniciar la actividad de opeeración de cálculo de costes hipotecarios
	 * @param v Vista actual
	 */	
    public void onCalculate(View v){
		// Se debe crear un intent con la nueva actividad y añadirle los parámetros que queremos que reciba
		// En nuestro caso son los parámetros introducidos por las barras de usuario.
        Intent i = new Intent(this, OperationActivity.class);
        i.putExtra("PRECIO", precioTV.getText().toString());
        i.putExtra("AHORRO", ahorroTV.getText().toString());
        i.putExtra("INTERES", interesTV.getText().toString());
        i.putExtra("PLAZO", plazoTV.getText().toString());
        startActivity(i);
    }

}
